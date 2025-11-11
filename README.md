# java-draco-decoder

Tiny CLI to **decode Draco-compressed GLB/GLTF** into plain **GLB 2.0** via LWJGL Assimp. Perfect pre-step before CPU voxelization.

## Requirements

* JDK 17+, Maven 3.8+
* LWJGL 3.3.2+ with Assimp (Draco bundled)

## Build

```bash
mvn -q -DskipTests clean package
# → target/draco-decoder-cli-1.0.0-all.jar
```

## Usage

```bash
java -jar target/draco-decoder-cli-1.0.0-all.jar \
  -f in.glb [-f in2.glb ...] \
  [-filelist list.txt] [-o out] [-j jobs] [-v] [-h]
```

**Options**

* `-f <path>` add input (repeatable)
* `-filelist <txt>` one path per line
* `-o <dir>` output dir (default `./out`)
* `-j <n>` parallel jobs (default: #CPU)
* `-v` verbose
* `-h` help

**Examples**

```bash
# single
java -jar ... -f tile.glb -o out
# batch
java -jar ... -f a.glb -f b.glb -o out -j 8
# list
java -jar ... -filelist assets.txt -o out
```

**Output**
Each `in.glb` → `out/in-decoded.glb` (no Draco).

## Notes

* Uses Assimp with safe defaults (`Triangulate`, `JoinIdenticalVertices`, etc.) and exports `glb2`.
* If you voxelize after: you can diff geometry vs CUDA output by sorting `.xyzi` arrays with `jq` and `diff`.

## Troubleshooting

* Missing natives / `UnsatisfiedLinkError`: ensure LWJGL Assimp natives for your OS are available.
* “Draco library not present” under `-v` is informational; if import works, you’re good.

## Embed (one-liner)

```java
AssimpDracoDecode.decodeToUncompressedGlb(new File("in.glb"), new File("out/in-decoded.glb"), true);
```
