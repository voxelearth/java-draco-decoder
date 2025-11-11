package com.example.draco;

import org.lwjgl.assimp.*;
import org.lwjgl.system.Configuration;
import org.lwjgl.system.SharedLibrary;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Imports a (possibly Draco-compressed) GLTF/GLB via Assimp and exports a plain GLB (no Draco).
 */
public final class AssimpDracoDecode {

    private AssimpDracoDecode() {}

    public static void decodeToUncompressedGlb(File input, File output, boolean verbose) throws Exception {
        if (!input.isFile()) throw new IllegalArgumentException("Input does not exist: " + input);

        // Optional: enable LWJGL/Assimp diagnostics when -v
        if (verbose) {
            Configuration.DISABLE_FUNCTION_CHECKS.set(false);
            Configuration.DEBUG.set(true);
        }

        // Ensure natives are loaded (Assimp + (if bundled) Draco)
        Assimp.getLibrary(); // load assimp native
        try {
            SharedLibrary draco = Assimp.getDraco(); // touch Draco library (if available)
            if (verbose) {
                System.out.println("[Native] Draco: " + draco.getName() + "  path=" + draco.getPath());
            }
        } catch (Throwable t) {
            if (verbose) System.out.println("[Native] Draco library not present (continuing): " + t);
        }

        // Typical post-process flags. Not strictly needed for "decode", but safe defaults.
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
            Path outPath = output.toPath();
            Files.createDirectories(outPath.getParent());

            // Exporter format id for binary glTF 2.0 is "glb2"
            int rc = Assimp.aiExportScene(scene, "glb2", outPath.toString(), 0);
            if (rc != Assimp.aiReturn_SUCCESS) {
                throw new IllegalStateException("Assimp export failed (rc=" + rc + ")");
            }
            if (verbose) System.out.println("[Export] Wrote " + outPath.toAbsolutePath());
        } finally {
            Assimp.aiReleaseImport(scene);
        }
    }
}
