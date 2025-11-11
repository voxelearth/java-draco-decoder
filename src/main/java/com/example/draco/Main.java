package com.example.draco;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;

public final class Main {

    private static void help() {
        System.out.println("""
            Draco → GLB decoder (+ rotate-flat bake like your Node script)

            Usage:
              java -jar draco-decoder-cli-1.0.0-all.jar -f <in.glb> [-f <in2.glb> ...]
                                                     [-filelist <list.txt>]
                                                     [-o <outdir>] [-j <jobs>] [-r] [--scale] [-v] [-h]

            Options:
              -f <path>            Input GLB/GLTF (repeatable). Draco OK.
              -filelist <txt>      Text file with one input path per line
              -o <dir>             Output directory (default ./out). Each file → <name>-decoded.glb
              -j <jobs>            Parallel jobs (default: #CPU)
              -r, --rotate-flat    Apply M' = RS * (T(-C) * M) to ROOTS, then:
                                   • bake each ROOT's current (R·S) into its own mesh vertices/normals/tangents(+morphs),
                                   • set that root to translation-only,
                                   • pre-multiply direct children by the same RS.
                                   (Exactly like your rotateUtils.js.)
              --scale              Include uniform scale S = 1/diag (world AABB diagonal).
              -v                   Verbose
              -h                   Show this help
            """);
    }

    public static void main(String[] args) {
        if (args.length == 0) { help(); System.exit(1); }

        List<File> inputs = new ArrayList<>();
        File outDir = new File("out");
        boolean verbose = false;
        int jobs = Math.max(1, Runtime.getRuntime().availableProcessors());
        boolean rotateFlat = false;
        boolean scaleOn = false;

        try {
            for (int i = 0; i < args.length; i++) {
                String a = args[i];
                switch (a) {
                    case "-h" -> { help(); return; }
                    case "-v" -> verbose = true;
                    case "-o" -> {
                        if (++i >= args.length) throw new IllegalArgumentException("Missing value for -o");
                        outDir = new File(args[i]);
                    }
                    case "-j" -> {
                        if (++i >= args.length) throw new IllegalArgumentException("Missing value for -j");
                        jobs = Math.max(1, Integer.parseInt(args[i]));
                    }
                    case "-f" -> {
                        if (++i >= args.length) throw new IllegalArgumentException("Missing value for -f");
                        inputs.add(new File(args[i]));
                    }
                    case "-filelist" -> {
                        if (++i >= args.length) throw new IllegalArgumentException("Missing value for -filelist");
                        File list = new File(args[i]);
                        for (String line : Files.readAllLines(list.toPath())) {
                            String p = line.trim();
                            if (!p.isEmpty()) inputs.add(new File(p));
                        }
                    }
                    case "-r", "--rotate-flat" -> rotateFlat = true;
                    case "--scale" -> scaleOn = true;
                    default -> {
                        if (a.startsWith("-")) {
                            System.err.println("Unknown option: " + a);
                            help();
                            System.exit(2);
                        } else {
                            inputs.add(new File(a)); // bare path convenience
                        }
                    }
                }
            }

            if (inputs.isEmpty()) {
                System.err.println("No input files.");
                help();
                System.exit(2);
            }
            if (!outDir.exists() && !outDir.mkdirs()) {
                System.err.println("Cannot create output dir: " + outDir.getAbsolutePath());
                System.exit(3);
            }

            System.out.println("[CLI] Files=" + inputs.size());
            System.out.println("[CLI] Output: " + outDir.getAbsolutePath());
            System.out.println("[CLI] Jobs=" + jobs + "  RotateFlat=" + rotateFlat + "  Scale=" + scaleOn + "  Verbose=" + verbose);

            final File outDirF = outDir;
            final boolean verboseF = verbose;
            final boolean rotateFlatF = rotateFlat;
            final boolean scaleOnF = scaleOn;

            ExecutorService pool = (jobs > 1)
                    ? Executors.newWorkStealingPool(jobs)
                    : Executors.newSingleThreadExecutor();

            List<Future<?>> futures = new ArrayList<>();
            long t0 = System.nanoTime();

            for (File in : inputs) {
                futures.add(pool.submit(() -> {
                    try {
                        String base = stripExt(in.getName());
                        File out = new File(outDirF, base + "-decoded.glb");
                        long t = System.nanoTime();
                        AssimpDracoDecode.decodeToUncompressedGlb(in, out, verboseF, rotateFlatF, scaleOnF);
                        long dt = System.nanoTime() - t;
                        System.out.printf("[OK] %-40s → %s  (%.1f ms)%n",
                                in.getName(), out.getName(), dt / 1e6);
                    } catch (Throwable e) {
                        System.err.printf("[ERR] %s: %s%n", in.getName(), e.getMessage());
                        e.printStackTrace(System.err);
                    }
                }));
            }

            for (Future<?> f : futures) f.get(); // wait all
            pool.shutdown();
            long t1 = System.nanoTime();
            System.out.printf("[ALL DONE] %d files in %.1f ms%n", inputs.size(), (t1 - t0) / 1e6);

        } catch (Throwable e) {
            System.err.println("[FATAL] " + e.getMessage());
            e.printStackTrace();
            System.exit(10);
        }
    }

    private static String stripExt(String s) {
        int i = s.lastIndexOf('.');
        return (i >= 0) ? s.substring(0, i) : s;
    }
}
