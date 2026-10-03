import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

// Tauri loads `dist/` in a release build and this dev server in `tauri dev`
// (`devUrl` in src-tauri/tauri.conf.json). The front end is built on the
// WSL side; the Rust half on Windows, where WebView2 is.
export default defineConfig({
  plugins: [svelte()],
  clearScreen: false,
  server: { port: 5173, strictPort: true },
  build: { target: 'es2022', outDir: 'dist', emptyOutDir: true },
});
