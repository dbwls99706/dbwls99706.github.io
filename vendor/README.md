# Vendored libraries

| File | Library | Version | License |
| --- | --- | --- | --- |
| `gsap.min.js`, `ScrollTrigger.min.js`, `SplitText.min.js`, `ScrambleTextPlugin.min.js` | [GSAP](https://gsap.com) | 3.15.1 | GSAP Standard License |
| `lenis.min.js` | [Lenis](https://github.com/darkroomengineering/lenis) | 1.3.27 | MIT |
| `ogl.min.js` | [OGL](https://github.com/oframe/ogl) | 1.0.11 | Unlicense |

`ogl.min.js` is an ES module bundle of `Renderer`, `Camera`, `Transform`, `Program`, `Geometry`, `Mesh` and `Texture`:

```bash
echo "export { Renderer, Camera, Transform, Program, Geometry, Mesh, Texture } from 'ogl';" > entry.js
npx esbuild entry.js --bundle --format=esm --minify --outfile=ogl.min.js
```
