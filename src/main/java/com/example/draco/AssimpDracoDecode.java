package com.example.draco;

import org.lwjgl.PointerBuffer;
import org.lwjgl.assimp.*;
import org.lwjgl.system.Configuration;
import org.lwjgl.system.SharedLibrary;

import java.io.File;
import java.nio.IntBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

/**
 * Draco → GLB decode.
 *
 * With -r/--rotate-flat:
 *   1) Compute world AABB center C from the ORIGINAL scene; geodeticUp = normalize(C).
 *   2) Build Q that rotates geodeticUp -> +Y (glTF is Y-up, right-handed). Optional S = 1/diag with --scale.
 *   3) Convert each mesh vertex from local→world using its node's world TRS, apply the global bake around C,
 *      then world→local back into the mesh so geometry really changes in-place.
 *   4) Normals/tangents are transformed with the correct normal matrices and renormalized.
 *   5) Export GLB. This guarantees a visible difference vs. the non-rotated path because geometry is rewritten.
 */
public final class AssimpDracoDecode {

    private AssimpDracoDecode() {}

    // ---------- Public entry points ----------

    public static void decodeToUncompressedGlb(File input, File output, boolean verbose) throws Exception {
        decodeToUncompressedGlb(input, output, verbose, false, false);
    }

    public static void decodeToUncompressedGlb(File input, File output, boolean verbose,
                                               boolean rotateFlat, boolean scaleOn) throws Exception {
        if (!input.isFile()) throw new IllegalArgumentException("Input does not exist: " + input);

        if (verbose) {
            Configuration.DISABLE_FUNCTION_CHECKS.set(false);
            Configuration.DEBUG.set(true);
        }

        // Load Assimp + Draco
        Assimp.getLibrary();
        try {
            SharedLibrary draco = Assimp.getDraco();
            if (verbose) {
                System.out.println("[Native] Draco: " + draco.getName() + "  path=" + draco.getPath());
            }
        } catch (Throwable t) {
            if (verbose) System.out.println("[Native] Draco library not present (continuing): " + t);
        }

        int ppFlags =
                Assimp.aiProcess_ValidateDataStructure |
                Assimp.aiProcess_Triangulate |
                Assimp.aiProcess_JoinIdenticalVertices |
                Assimp.aiProcess_ImproveCacheLocality |
                Assimp.aiProcess_SortByPType;

        if (verbose) System.out.println("[Import] " + input.getAbsolutePath());

        AIScene scene = Assimp.aiImportFile(input.getAbsolutePath(), ppFlags);
        if (scene == null) {
            throw new IllegalStateException("Assimp import failed: " + Assimp.aiGetErrorString());
        }

        try {
            if (rotateFlat) {
                rotateFlatByPretransformingAndBaking(scene, verbose, scaleOn);
            }

            Path outPath = output.toPath();
            Files.createDirectories(outPath.getParent());

            int rc = Assimp.aiExportScene(scene, "glb2", outPath.toString(), 0);
            if (rc != Assimp.aiReturn_SUCCESS) {
                throw new IllegalStateException("Assimp export failed (rc=" + rc + ")");
            }
            if (verbose) System.out.println("[Export] Wrote " + outPath.toAbsolutePath());

        } finally {
            Assimp.aiReleaseImport(scene);
        }
    }

    // ---------- Rotate-flat by baking into vertices (robust/visible) ----------
    private static void rotateFlatByPretransformingAndBaking(AIScene scene, boolean verbose, boolean scaleOn) {
        if (scene.mRootNode() == null) return;

        // 1) Compute global center/diag from the ORIGINAL hierarchy
        WorldBBox bbox = worldBBoxFromOriginal(scene);
        float[] C = bbox.center;
        double diag = bbox.diag;

        if (verbose) {
            System.out.printf("[BBox] center=(%.6f,%.6f,%.6f) diag=%.6f%n", C[0], C[1], C[2], diag);
        }

        // 2) geodeticUp = normalize(C) → rotate to +Y (glTF is Y-up, right-handed)
        float[] up = normalize3(C);
        if (!isFinite3(up)) up = new float[]{0f, 1f, 0f};
        float[] q = quatFromUnitVectors(up, new float[]{0f, 1f, 0f});   // rotate "up" to +Y
        float s = scaleOn ? (diag > 0 ? (float)(1.0 / diag) : 1f) : 1f;
        float[] R = quatToMat3(q); // 3x3 rotation for global bake

        // 3) Build world matrices (column-major) for all nodes
        Map<AINode, float[]> world = new java.util.HashMap<>();
        buildWorld(scene.mRootNode(), identity4(), world);

        // 4) Walk nodes and bake rotation into each attached mesh
        PointerBuffer meshesBuf = scene.mMeshes();

        class Walker {
            void walk(AINode node) {
                float[] W = world.get(node);              // world 4x4 (CM)
                if (W == null) W = identity4();

                // Extract 3x3 (rotation*scale) and translation
                float[] F = new float[]{ W[0],W[1],W[2],  W[4],W[5],W[6],  W[8],W[9],W[10] }; // upper-left
                float[] Ft = mat3Transpose(F);
                float[] Finv = mat3Inverse(F);            // may be null if degenerate
                if (Finv == null) Finv = new float[]{1,0,0, 0,1,0, 0,0,1};
                float[] FinvT = mat3Transpose(Finv);
                float tx = W[12], ty = W[13], tz = W[14];

                // Decompose W to TRS to invert positions back to local cleanly:
                // v_local = S^{-1} * Rw^T * (v_world - t)
                TRS wtrs = decomposeTRS(W);              // r is unit quaternion; s are per-axis scales
                float[] Rw = quatToMat3(wtrs.r);
                float[] RwT = mat3Transpose(Rw);
                float isx = (wtrs.s[0] != 0f) ? (1f / wtrs.s[0]) : 0f;
                float isy = (wtrs.s[1] != 0f) ? (1f / wtrs.s[1]) : 0f;
                float isz = (wtrs.s[2] != 0f) ? (1f / wtrs.s[2]) : 0f;

                IntBuffer meshIdx = node.mMeshes();
                if (meshIdx != null) {
                    for (int k = 0; k < node.mNumMeshes(); k++) {
                        int mi = meshIdx.get(k);
                        AIMesh mesh = AIMesh.create(meshesBuf.get(mi));
                        if (mesh == null) continue;

                        // --- Positions ---
                        AIVector3D.Buffer vtx = mesh.mVertices();
                        if (vtx != null) {
                            for (int i = 0; i < mesh.mNumVertices(); i++) {
                                AIVector3D p = vtx.get(i);

                                // local -> world : pW = F * p + t
                                float lx = p.x(), ly = p.y(), lz = p.z();
                                float pwx = F[0]*lx + F[3]*ly + F[6]*lz + tx;
                                float pwy = F[1]*lx + F[4]*ly + F[7]*lz + ty;
                                float pwz = F[2]*lx + F[5]*ly + F[8]*lz + tz;

                                // global bake around C: pW' = C + R * (pW - C) * s
                                float dx = pwx - C[0], dy = pwy - C[1], dz = pwz - C[2];
                                float rx = R[0]*dx + R[3]*dy + R[6]*dz;
                                float ry = R[1]*dx + R[4]*dy + R[7]*dz;
                                float rz = R[2]*dx + R[5]*dy + R[8]*dz;
                                float ppx = C[0] + rx * s;
                                float ppy = C[1] + ry * s;
                                float ppz = C[2] + rz * s;

                                // world -> local : v_local = S^{-1} * Rw^T * (pW' - t)
                                float vx = ppx - wtrs.t[0];
                                float vy = ppy - wtrs.t[1];
                                float vz = ppz - wtrs.t[2];
                                float v0 = RwT[0]*vx + RwT[3]*vy + RwT[6]*vz;
                                float v1 = RwT[1]*vx + RwT[4]*vy + RwT[7]*vz;
                                float v2 = RwT[2]*vx + RwT[5]*vy + RwT[8]*vz;
                                p.set(v0*isx, v1*isy, v2*isz);
                            }
                        }

                        // --- Normals (use normal matrices) ---
                        AIVector3D.Buffer nrm = mesh.mNormals();
                        if (nrm != null) {
                            for (int i = 0; i < mesh.mNumVertices(); i++) {
                                AIVector3D n = nrm.get(i);

                                // local -> world normals: nW = (F^-1)^T * nL
                                float nx = n.x(), ny = n.y(), nz = n.z();
                                float nwx = FinvT[0]*nx + FinvT[3]*ny + FinvT[6]*nz;
                                float nwy = FinvT[1]*nx + FinvT[4]*ny + FinvT[7]*nz;
                                float nwz = FinvT[2]*nx + FinvT[5]*ny + FinvT[8]*nz;
                                float nwl = (float)Math.sqrt(nwx*nwx + nwy*nwy + nwz*nwz);
                                if (nwl > 0f) { nwx/=nwl; nwy/=nwl; nwz/=nwl; }

                                // apply global rotate: nW' = R * nW
                                float nrx = R[0]*nwx + R[3]*nwy + R[6]*nwz;
                                float nry = R[1]*nwx + R[4]*nwy + R[7]*nwz;
                                float nrz = R[2]*nwx + R[5]*nwy + R[8]*nwz;

                                // world -> local normals: nL' = (F)^T * nW'
                                float nlx = Ft[0]*nrx + Ft[3]*nry + Ft[6]*nrz;
                                float nly = Ft[1]*nrx + Ft[4]*nry + Ft[7]*nrz;
                                float nlz = Ft[2]*nrx + Ft[5]*nry + Ft[8]*nrz;
                                float nl = (float)Math.sqrt(nlx*nlx + nly*nly + nlz*nlz);
                                if (nl > 0f) { nlx/=nl; nly/=nl; nlz/=nl; }
                                n.set(nlx, nly, nlz);
                            }
                        }

                        // --- Tangents (same normal matrices) ---
                        AIVector3D.Buffer tan = mesh.mTangents();
                        if (tan != null) {
                            for (int i = 0; i < mesh.mNumVertices(); i++) {
                                AIVector3D t = tan.get(i);
                                float tx0 = t.x(), ty0 = t.y(), tz0 = t.z();

                                float twx = FinvT[0]*tx0 + FinvT[3]*ty0 + FinvT[6]*tz0;
                                float twy = FinvT[1]*tx0 + FinvT[4]*ty0 + FinvT[7]*tz0;
                                float twz = FinvT[2]*tx0 + FinvT[5]*ty0 + FinvT[8]*tz0;
                                float twl = (float)Math.sqrt(twx*twx + twy*twy + twz*twz);
                                if (twl > 0f) { twx/=twl; twy/=twl; twz/=twl; }

                                float trx = R[0]*twx + R[3]*twy + R[6]*twz;
                                float tryy= R[1]*twx + R[4]*twy + R[7]*twz;
                                float trz = R[2]*twx + R[5]*twy + R[8]*twz;

                                float tlx = Ft[0]*trx + Ft[3]*tryy + Ft[6]*trz;
                                float tly = Ft[1]*trx + Ft[4]*tryy + Ft[7]*trz;
                                float tlz = Ft[2]*trx + Ft[5]*tryy + Ft[8]*trz;
                                float tl = (float)Math.sqrt(tlx*tlx + tly*tly + tlz*tlz);
                                if (tl > 0f) { tlx/=tl; tly/=tl; tlz/=tl; }
                                t.set(tlx, tly, tlz);
                            }
                        }

                        // Morph targets (AIAnimMesh): apply same logic to positions/normals/tangents
                        PointerBuffer anims = mesh.mAnimMeshes();
                        if (anims != null) {
                            for (int j = 0; j < mesh.mNumAnimMeshes(); j++) {
                                AIAnimMesh am = AIAnimMesh.create(anims.get(j));

                                AIVector3D.Buffer mv = am.mVertices();
                                if (mv != null) {
                                    for (int i = 0; i < am.mNumVertices(); i++) {
                                        AIVector3D p = mv.get(i);
                                        float lx = p.x(), ly = p.y(), lz = p.z();
                                        float pwx = F[0]*lx + F[3]*ly + F[6]*lz + tx;
                                        float pwy = F[1]*lx + F[4]*ly + F[7]*lz + ty;
                                        float pwz = F[2]*lx + F[5]*ly + F[8]*lz + tz;
                                        float dx = pwx - C[0], dy = pwy - C[1], dz = pwz - C[2];
                                        float rx = R[0]*dx + R[3]*dy + R[6]*dz;
                                        float ry = R[1]*dx + R[4]*dy + R[7]*dz;
                                        float rz = R[2]*dx + R[5]*dy + R[8]*dz;
                                        float ppx = C[0] + rx * s;
                                        float ppy = C[1] + ry * s;
                                        float ppz = C[2] + rz * s;
                                        float vx = ppx - wtrs.t[0];
                                        float vy = ppy - wtrs.t[1];
                                        float vz = ppz - wtrs.t[2];
                                        float v0 = RwT[0]*vx + RwT[3]*vy + RwT[6]*vz;
                                        float v1 = RwT[1]*vx + RwT[4]*vy + RwT[7]*vz;
                                        float v2 = RwT[2]*vx + RwT[5]*vy + RwT[8]*vz;
                                        p.set(v0*isx, v1*isy, v2*isz);
                                    }
                                }
                                AIVector3D.Buffer mn = am.mNormals();
                                if (mn != null) {
                                    for (int i = 0; i < am.mNumVertices(); i++) {
                                        AIVector3D n = mn.get(i);
                                        float nx = n.x(), ny = n.y(), nz = n.z();
                                        float nwx = FinvT[0]*nx + FinvT[3]*ny + FinvT[6]*nz;
                                        float nwy = FinvT[1]*nx + FinvT[4]*ny + FinvT[7]*nz;
                                        float nwz = FinvT[2]*nx + FinvT[5]*ny + FinvT[8]*nz;
                                        float nwl = (float)Math.sqrt(nwx*nwx + nwy*nwy + nwz*nwz);
                                        if (nwl > 0f) { nwx/=nwl; nwy/=nwl; nwz/=nwl; }
                                        float nrx = R[0]*nwx + R[3]*nwy + R[6]*nwz;
                                        float nry = R[1]*nwx + R[4]*nwy + R[7]*nwz;
                                        float nrz = R[2]*nwx + R[5]*nwy + R[8]*nwz;
                                        float nlx = Ft[0]*nrx + Ft[3]*nry + Ft[6]*nrz;
                                        float nly = Ft[1]*nrx + Ft[4]*nry + Ft[7]*nrz;
                                        float nlz = Ft[2]*nrx + Ft[5]*nry + Ft[8]*nrz;
                                        float nl = (float)Math.sqrt(nlx*nlx + nly*nly + nlz*nlz);
                                        if (nl > 0f) { nlx/=nl; nly/=nl; nlz/=nl; }
                                        n.set(nlx, nly, nlz);
                                    }
                                }
                                AIVector3D.Buffer mt = am.mTangents();
                                if (mt != null) {
                                    for (int i = 0; i < am.mNumVertices(); i++) {
                                        AIVector3D t = mt.get(i);
                                        float tx0 = t.x(), ty0 = t.y(), tz0 = t.z();
                                        float twx = FinvT[0]*tx0 + FinvT[3]*ty0 + FinvT[6]*tz0;
                                        float twy = FinvT[1]*tx0 + FinvT[4]*ty0 + FinvT[7]*tz0;
                                        float twz = FinvT[2]*tx0 + FinvT[5]*ty0 + FinvT[8]*tz0;
                                        float twl = (float)Math.sqrt(twx*twx + twy*twy + twz*twz);
                                        if (twl > 0f) { twx/=twl; twy/=twl; twz/=twl; }
                                        float trx = R[0]*twx + R[3]*twy + R[6]*twz;
                                        float tryy= R[1]*twx + R[4]*twy + R[7]*twz;
                                        float trz = R[2]*twx + R[5]*twy + R[8]*twz;
                                        float tlx = Ft[0]*trx + Ft[3]*tryy + Ft[6]*trz;
                                        float tly = Ft[1]*trx + Ft[4]*tryy + Ft[7]*trz;
                                        float tlz = Ft[2]*trx + Ft[5]*tryy + Ft[8]*trz;
                                        float tl = (float)Math.sqrt(tlx*tlx + tly*tly + tlz*tlz);
                                        if (tl > 0f) { tlx/=tl; tly/=tl; tlz/=tl; }
                                        t.set(tlx, tly, tlz);
                                    }
                                }
                            }
                        }
                    }
                }

                PointerBuffer kids = node.mChildren();
                for (int i = 0; i < node.mNumChildren(); i++) walk(AINode.create(kids.get(i)));
            }
        }
        new Walker().walk(scene.mRootNode());

        if (verbose) {
            float[] first = firstVertex(scene);
            if (first != null) {
                System.out.printf("[BakeCheck] first vertex after bake = (%.6f, %.6f, %.6f)%n",
                        first[0], first[1], first[2]);
            }
            System.out.println("[RotateFlat] In-place bake completed (geometry rewritten).");
        }
    }

    private static float[] firstVertex(AIScene scene) {
        PointerBuffer meshes = scene.mMeshes();
        for (int mi = 0; mi < scene.mNumMeshes(); mi++) {
            AIMesh mesh = AIMesh.create(meshes.get(mi));
            if (mesh != null && mesh.mNumVertices() > 0 && mesh.mVertices() != null) {
                AIVector3D p = mesh.mVertices().get(0);
                return new float[]{p.x(), p.y(), p.z()};
            }
        }
        return null;
    }

    // ---------- World bbox from ORIGINAL hierarchy ----------

    private record WorldBBox(float[] center, double diag) {}

    private static WorldBBox worldBBoxFromOriginal(AIScene scene) {
        // Build world matrices by walking the hierarchy (row-major in Assimp → we convert to column-major)
        MinMax mm = new MinMax();
        buildAndAccumulate(scene.mRootNode(), identity4(), scene.mMeshes(), mm);

        if (!mm.valid) {
            mm.minX = mm.minY = mm.minZ = -0.5;
            mm.maxX = mm.maxY = mm.maxZ =  0.5;
        }

        float cx = (float)((mm.minX + mm.maxX)/2.0);
        float cy = (float)((mm.minY + mm.maxY)/2.0);
        float cz = (float)((mm.minZ + mm.maxZ)/2.0);

        double dx = mm.maxX - mm.minX, dy = mm.maxY - mm.minY, dz = mm.maxZ - mm.minZ;
        double diag = Math.sqrt(dx*dx + dy*dy + dz*dz);
        return new WorldBBox(new float[]{cx, cy, cz}, diag);
    }

    private static final class MinMax {
        boolean valid = false;
        double minX =  Double.POSITIVE_INFINITY, minY =  Double.POSITIVE_INFINITY, minZ =  Double.POSITIVE_INFINITY;
        double maxX =  Double.NEGATIVE_INFINITY, maxY =  Double.NEGATIVE_INFINITY, maxZ =  Double.NEGATIVE_INFINITY;
        void grow(double mnx, double mny, double mnz, double mxx, double mxy, double mxz) {
            if (mnx < minX) minX = mnx; if (mny < minY) minY = mny; if (mnz < minZ) minZ = mnz;
            if (mxx > maxX) maxX = mxx; if (mxy > maxY) maxY = mxy; if (mxz > maxZ) maxZ = mxz;
            valid = true;
        }
    }

    private static void buildAndAccumulate(AINode node, float[] parentCM, PointerBuffer meshesBuf, MinMax mm) {
        float[] localCM = toCM(node.mTransformation());
        float[] worldCM = mat4Mul(parentCM, localCM);

        IntBuffer idx = node.mMeshes();
        if (idx != null) {
            for (int k = 0; k < node.mNumMeshes(); k++) {
                int mi = idx.get(k);
                AIMesh mesh = AIMesh.create(meshesBuf.get(mi));
                if (mesh == null) continue;

                AIVector3D.Buffer v = mesh.mVertices();
                if (v == null || mesh.mNumVertices() == 0) continue;

                // local AABB
                float lminX = Float.POSITIVE_INFINITY, lminY = Float.POSITIVE_INFINITY, lminZ = Float.POSITIVE_INFINITY;
                float lmaxX = Float.NEGATIVE_INFINITY, lmaxY = Float.NEGATIVE_INFINITY, lmaxZ = Float.NEGATIVE_INFINITY;
                for (int i = 0; i < mesh.mNumVertices(); i++) {
                    AIVector3D p = v.get(i);
                    float x = p.x(), y = p.y(), z = p.z();
                    if (x < lminX) lminX = x; if (y < lminY) lminY = y; if (z < lminZ) lminZ = z;
                    if (x > lmaxX) lmaxX = x; if (y > lmaxY) lmaxY = y; if (z > lmaxZ) lmaxZ = z;
                }
                float[] cLocal = new float[]{(lminX+lmaxX)/2f, (lminY+lmaxY)/2f, (lminZ+lmaxZ)/2f};
                float[] hLocal = new float[]{(lmaxX-lminX)/2f, (lmaxY-lminY)/2f, (lmaxZ-lminZ)/2f};

                AABB tr = transformAabb(cLocal, hLocal, worldCM);
                float cx = tr.center[0], cy = tr.center[1], cz = tr.center[2];
                float hx = tr.half[0],   hy = tr.half[1],   hz = tr.half[2];

                double mnx = cx - hx, mny = cy - hy, mnz = cz - hz;
                double mxx = cx + hx, mxy = cy + hy, mxz = cz + hz;

                mm.grow(mnx, mny, mnz, mxx, mxy, mxz);
            }
        }

        PointerBuffer kids = node.mChildren();
        for (int i = 0; i < node.mNumChildren(); i++) {
            buildAndAccumulate(AINode.create(kids.get(i)), worldCM, meshesBuf, mm);
        }
    }

    private record AABB(float[] center, float[] half) {}

    // Transform AABB by |R| and add T (column-major worldCM)
    private static AABB transformAabb(float[] center, float[] half, float[] M) {
        float m00=M[0], m01=M[4], m02=M[8];
        float m10=M[1], m11=M[5], m12=M[9];
        float m20=M[2], m21=M[6], m22=M[10];
        float tx = M[12], ty = M[13], tz = M[14];

        float cx = m00*center[0] + m01*center[1] + m02*center[2] + tx;
        float cy = m10*center[0] + m11*center[1] + m12*center[2] + ty;
        float cz = m20*center[0] + m21*center[1] + m22*center[2] + tz;

        float ax00 = Math.abs(m00), ax01 = Math.abs(m01), ax02 = Math.abs(m02);
        float ax10 = Math.abs(m10), ax11 = Math.abs(m11), ax12 = Math.abs(m12);
        float ax20 = Math.abs(m20), ax21 = Math.abs(m21), ax22 = Math.abs(m22);

        float hx = ax00*half[0] + ax01*half[1] + ax02*half[2];
        float hy = ax10*half[0] + ax11*half[1] + ax12*half[2];
        float hz = ax20*half[0] + ax21*half[1] + ax22*half[2];

        return new AABB(new float[]{cx,cy,cz}, new float[]{hx,hy,hz});
    }

    // ---------- Math (column-major; matches the Node helpers) ----------

    private static boolean isFinite3(float[] v) {
        return Float.isFinite(v[0]) && Float.isFinite(v[1]) && Float.isFinite(v[2]);
    }

    private static float[] identity4() {
        return new float[]{1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1};
    }

    private static float[] normalize3(float[] v) {
        double L = Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (!(L > 0)) return new float[]{0,0,0};
        return new float[]{(float)(v[0]/L), (float)(v[1]/L), (float)(v[2]/L)};
    }

    private static float clamp(float x, float lo, float hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static float dot(float[] a, float[] b) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    private static float[] cross(float[] a, float[] b) {
        return new float[]{
                a[1]*b[2]-a[2]*b[1],
                a[2]*b[0]-a[0]*b[2],
                a[0]*b[1]-a[1]*b[0]
        };
    }

    private static float[] quatFromUnitVectors(float[] from, float[] to) {
        float d = clamp(dot(from, to), -1f, 1f);
        if (d > 0.999999f) return new float[]{0,0,0,1};
        if (d < -0.999999f) {
            float[] axis = cross(new float[]{1,0,0}, from);
            float L = (float)Math.sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
            if (L < 1e-5f) axis = cross(new float[]{0,1,0}, from);
            axis = normalize3(axis);
            float s = (float)Math.sin(Math.PI/2.0), c = (float)Math.cos(Math.PI/2.0);
            return new float[]{axis[0]*s, axis[1]*s, axis[2]*s, c};
        }
        float[] axis = cross(from, to);
        float s = (float)Math.sqrt((1+d)*2);
        float invs = 1f / s;
        return new float[]{axis[0]*invs, axis[1]*invs, axis[2]*invs, s*0.5f};
    }

    private static float[] quatToMat3(float[] q) {
        float x=q[0], y=q[1], z=q[2], w=q[3];
        float x2=x+x, y2=y+y, z2=z+z;
        float xx=x*x2, yy=y*y2, zz=z*z2;
        float xy=x*y2, xz=x*z2, yz=y*z2;
        float wx=w*x2, wy=w*y2, wz=w*z2;
        return new float[]{
                1-(yy+zz),  xy-wz,     xz+wy,
                xy+wz,      1-(xx+zz), yz-wx,
                xz-wy,      yz+wx,     1-(xx+yy),
        };
    }

    // 3x3 transpose
    private static float[] mat3Transpose(float[] m) {
        // column-major packed 3x3
        return new float[]{
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        };
    }

    // 3x3 inverse (column-major layout)
    private static float[] mat3Inverse(float[] m) {
        float a00=m[0], a01=m[3], a02=m[6];
        float a10=m[1], a11=m[4], a12=m[7];
        float a20=m[2], a21=m[5], a22=m[8];

        float b01 =  a22*a11 - a12*a21;
        float b11 = -a22*a10 + a12*a20;
        float b21 =  a21*a10 - a11*a20;

        float det = a00*b01 + a01*b11 + a02*b21;
        if (!Float.isFinite(det) || Math.abs(det) < 1e-20f) return null;
        float inv = 1f / det;

        return new float[]{
            b01*inv,
            (-a22*a01 + a02*a21)*inv,
            ( a12*a01 - a02*a11)*inv,

            b11*inv,
            ( a22*a00 - a02*a20)*inv,
            (-a12*a00 + a02*a10)*inv,

            b21*inv,
            (-a21*a00 + a01*a20)*inv,
            ( a11*a00 - a01*a10)*inv
        };
    }

    // Decompose a column-major 4x4 into translation t, unit quaternion r, and per-axis scale s
    private static TRS decomposeTRS(float[] M) {
        float[] t = new float[]{ M[12], M[13], M[14] };

        float sx = (float)Math.hypot(M[0], Math.hypot(M[1], M[2]));  if (sx == 0) sx = 1;
        float sy = (float)Math.hypot(M[4], Math.hypot(M[5], M[6]));  if (sy == 0) sy = 1;
        float sz = (float)Math.hypot(M[8], Math.hypot(M[9], M[10])); if (sz == 0) sz = 1;

        float r00 = M[0]/sx, r01 = M[4]/sy, r02 = M[8]/sz;
        float r10 = M[1]/sx, r11 = M[5]/sy, r12 = M[9]/sz;
        float r20 = M[2]/sx, r21 = M[6]/sy, r22 = M[10]/sz;

        float trace = r00 + r11 + r22;
        float[] q;
        if (trace > 0f) {
            float s = (float)Math.sqrt(trace + 1f) * 2f; // 4*qw
            q = new float[]{ (r21-r12)/s, (r02-r20)/s, (r10-r01)/s, 0.25f*s };
        } else if (r00 > r11 && r00 > r22) {
            float s = (float)Math.sqrt(1f + r00 - r11 - r22) * 2f; // 4*qx
            q = new float[]{ 0.25f*s, (r01+r10)/s, (r02+r20)/s, (r21-r12)/s };
        } else if (r11 > r22) {
            float s = (float)Math.sqrt(1f + r11 - r00 - r22) * 2f; // 4*qy
            q = new float[]{ (r01+r10)/s, 0.25f*s, (r12+r21)/s, (r02-r20)/s };
        } else {
            float s = (float)Math.sqrt(1f + r22 - r00 - r11) * 2f; // 4*qz
            q = new float[]{ (r02+r20)/s, (r12+r21)/s, 0.25f*s, (r10-r01)/s };
        }
        return new TRS(t, q, new float[]{sx, sy, sz});
    }

    private static final class TRS {
        final float[] t, r, s;
        TRS(float[] t, float[] r, float[] s) { this.t=t; this.r=r; this.s=s; }
    }

    // ---------- Column-major <-> Assimp row-major ----------

    private static float[] toCM(AIMatrix4x4 src) {
        // Row-major (Assimp) → column-major float[16]
        return new float[]{
                src.a1(), src.b1(), src.c1(), src.d1(),
                src.a2(), src.b2(), src.c2(), src.d2(),
                src.a3(), src.b3(), src.c3(), src.d3(),
                src.a4(), src.b4(), src.c4(), src.d4()
        };
    }

    // Build world matrices (column-major) for every node
    private static void buildWorld(AINode node, float[] parentCM, java.util.Map<AINode, float[]> out) {
        float[] localCM = toCM(node.mTransformation());         // row-major (Assimp) → column-major
        float[] worldCM = mat4Mul(parentCM, localCM);           // parent ∘ local
        out.put(node, worldCM);

        PointerBuffer kids = node.mChildren();
        for (int i = 0; i < node.mNumChildren(); i++) {
            buildWorld(AINode.create(kids.get(i)), worldCM, out);
        }
    }

    // Multiply (column-major) a*b
    private static float[] mat4Mul(float[] a, float[] b) {
        float[] out = new float[16];
        for (int r=0; r<4; r++) {
            for (int c=0; c<4; c++) {
                out[c*4+r] = a[0*4+r]*b[c*4+0] + a[1*4+r]*b[c*4+1] + a[2*4+r]*b[c*4+2] + a[3*4+r]*b[c*4+3];
            }
        }
        return out;
    }
}
