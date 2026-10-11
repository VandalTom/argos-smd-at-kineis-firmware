// Assemble the static site for Cloudflare (wrangler.jsonc assets -> ./dist).
// The tool is a single self-contained HTML file kept with the other host
// tools; this copies it in as index.html and adds response headers.
// No dependencies: `node build.mjs` (npm run build) from this directory.

import { copyFileSync, mkdirSync, rmSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const src = join(here, '..', '..', 'Tools', 'naiad_tool.html');
const dist = join(here, 'dist');

rmSync(dist, { recursive: true, force: true });
mkdirSync(dist, { recursive: true });
copyFileSync(src, join(dist, 'index.html'));

// Workers static assets honour a _headers file. Serial access stays limited
// to this origin; the page needs no other powerful features.
writeFileSync(join(dist, '_headers'), [
	'/*',
	'  X-Content-Type-Options: nosniff',
	'  Referrer-Policy: no-referrer',
	'  Permissions-Policy: serial=(self), camera=(), microphone=(), geolocation=()',
	'  X-Frame-Options: DENY',
	'',
].join('\n'));

console.log(`built ${join(dist, 'index.html')}`);
