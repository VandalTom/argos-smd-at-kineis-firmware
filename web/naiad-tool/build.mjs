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

// Workers static assets honour a _headers file. Serial, USB and HID access
// (UART link, SWD probes) stay limited to this origin; the page needs no other powerful features. The CSP lets
// the single-file page run and talk to its own Worker API only (connect-src
// 'self': release list/images, team credentials store); no other host, no
// forms, no framing, so nothing typed into the page can be sent elsewhere.
const csp = [
	"default-src 'none'",
	"script-src 'self' 'unsafe-inline'",
	"style-src 'self' 'unsafe-inline'",
	"img-src 'self' data:",
	"connect-src 'self'",
	"form-action 'none'",
	"base-uri 'none'",
	"frame-ancestors 'none'",
].join('; ');
writeFileSync(join(dist, '_headers'), [
	'/*',
	'  X-Content-Type-Options: nosniff',
	'  Referrer-Policy: no-referrer',
	'  Permissions-Policy: serial=(self), usb=(self), hid=(self), camera=(), microphone=(), geolocation=()',
	'  X-Frame-Options: DENY',
	`  Content-Security-Policy: ${csp}`,
	'',
].join('\n'));

console.log(`built ${join(dist, 'index.html')}`);
