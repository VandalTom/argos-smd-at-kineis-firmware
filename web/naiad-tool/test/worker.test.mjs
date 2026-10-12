#!/usr/bin/env node
/**
 * Offline test of the Naiad tool Worker (src/worker.js): release listing and image proxy, the
 * Cloudflare Access check, and the team vault, with D1 played by node:sqlite and
 * GitHub / Access played by a mocked fetch. Everything goes through worker.fetch(), as on Cloudflare.
 *
 *   cd web/naiad-tool && npm test        (node 22.13+; exit code 0 = all passed)
 */

import { DatabaseSync } from 'node:sqlite';
import { crc32 as zcrc32 } from 'node:zlib';
import worker from '../src/worker.js';

/* ---------- D1 on node:sqlite ---------- */
class Stmt {
	constructor(db, sql, params = []) { this.db = db; this.sql = sql; this.params = params; }
	bind(...p) { return new Stmt(this.db, this.sql, p); }
	exec() {
		const s = this.db.prepare(this.sql);
		if (/^\s*SELECT/i.test(this.sql)) return { results: s.all(...this.params), success: true, meta: {} };
		return { results: [], success: true, meta: { changes: Number(s.run(...this.params).changes) } };
	}
	async first() { return this.db.prepare(this.sql).get(...this.params) ?? null; }
	async all() { return this.exec(); }
	async run() { return this.exec(); }
}
class D1 {
	constructor() { this.db = new DatabaseSync(':memory:'); }
	prepare(sql) { return new Stmt(this.db, sql); }
	async batch(list) {
		this.db.exec('BEGIN');
		try { const out = list.map(s => s.exec()); this.db.exec('COMMIT'); return out; }
		catch (e) { this.db.exec('ROLLBACK'); throw e; }
	}
}

/* ---------- Access: an RSA key pair standing in for the team's signing key ---------- */
const TEAM = 'naiadtest.cloudflareaccess.com', AUD = 'aud-0123456789abcdef', KID = 'kid-1';
const enc = s => Buffer.from(s).toString('base64url');
const rsa = () => crypto.subtle.generateKey({ name: 'RSASSA-PKCS1-v1_5', modulusLength: 2048, publicExponent: new Uint8Array([1, 0, 1]), hash: 'SHA-256' }, true, ['sign', 'verify']);
const teamKey = await rsa(), rogueKey = await rsa();
const teamJwk = { ...(await crypto.subtle.exportKey('jwk', teamKey.publicKey)), kid: KID, alg: 'RS256', use: 'sig' };
async function jwt(claims = {}, { key = teamKey, header = {} } = {}) {
	const now = Math.floor(Date.now() / 1000);
	const h = enc(JSON.stringify({ alg: 'RS256', kid: KID, typ: 'JWT', ...header }));
	const p = enc(JSON.stringify({ aud: [AUD], iss: `https://${TEAM}`, email: 'tom@siliconvandals.com', iat: now, nbf: now, exp: now + 3600, ...claims }));
	const sig = new Uint8Array(await crypto.subtle.sign('RSASSA-PKCS1-v1_5', key.privateKey, new TextEncoder().encode(`${h}.${p}`)));
	return `${h}.${p}.${Buffer.from(sig).toString('base64url')}`;
}

/* ---------- GitHub fixtures + mocked fetch ---------- */
const REPO = 'VandalTom/argos-smd-at-kineis-firmware';
const asset = (variant, ext = '.bin') => ({ name: `naiad-carrier-${variant}__argos-smd-at-kineis-firmware${ext}`, size: 123456 });
const GH_RELEASES = [
	{ tag_name: 'v1.0.1', name: 'v1.0.1', draft: false, prerelease: false, published_at: '2026-10-12T09:00:00Z', html_url: 'https://github.com/x/r/v1.0.1',
		assets: [asset('release'), asset('bench'), asset('release', '.elf'), asset('release', '_full.bin'), { name: 'gui-pa-uart__argos-smd-at-kineis-firmware.bin', size: 1 }] },
	{ tag_name: 'v1.1.0', draft: true, assets: [asset('release')] },
	{ tag_name: 'nightly', draft: false, assets: [asset('release')] },
	{ tag_name: 'v0.9.0', draft: false, assets: [{ name: 'gui-pa-uart__argos-smd-at-kineis-firmware.bin', size: 1 }] },
	{ tag_name: 'v1.0.0', draft: false, prerelease: false, published_at: '2026-10-11T09:00:00Z', assets: [asset('release'), asset('bench')] },
];
// The feed also carries plain tags (the fork inherits upstream's v3.x): only tags with Naiad images count
const feedEntry = (tag, date) => `<entry><id>tag:github.com,2008:Repository/1/${tag}</id><updated>${date}</updated>
<link rel="alternate" type="text/html" href="https://github.com/${REPO}/releases/tag/${tag}"/><title>${tag}</title></entry>`;
const ATOM = `<?xml version="1.0"?><feed>${feedEntry('v3.9.1', '2025-01-01T00:00:00Z')}${feedEntry('v1.0.1', '2026-10-12T09:00:00Z')}
${feedEntry('v3.10.0', '2025-02-01T00:00:00Z')}${feedEntry('nightly', '2026-10-01T00:00:00Z')}${feedEntry('v1.0.0', '2026-10-11T09:00:00Z')}</feed>`;
const ATTACHED = { 'v1.0.1': ['release', 'bench'], 'v1.0.0': ['release'] };
const IMAGE = new Uint8Array(4096).map((_, i) => i & 0xFF);
const net = { apiStatus: 200, certFetches: 0, log: [] };
globalThis.fetch = async (input, init = {}) => {
	const u = String(input instanceof Request ? input.url : input);
	net.log.push(u);
	if (u === `https://${TEAM}/cdn-cgi/access/certs`) { net.certFetches++; return Response.json({ keys: [teamJwk], public_cert: {}, public_certs: [] }); }
	if (u.startsWith(`https://api.github.com/repos/${REPO}/releases`)) return net.apiStatus === 200 ? Response.json(GH_RELEASES) : new Response('rate limited', { status: net.apiStatus });
	if (u === `https://github.com/${REPO}/releases.atom`) return new Response(ATOM);
	const dl = u.match(new RegExp(`^https://github.com/${REPO}/releases/download/(v[\\d.]+)/(.+)$`));
	if (dl && init.method === 'HEAD') {                // GitHub redirects an attached asset to its storage
		const variant = (dl[2].match(/^naiad-carrier-(\w+)__/) || [])[1];
		return (ATTACHED[dl[1]] || []).includes(variant) ? new Response(null, { status: 302, headers: { location: 'https://objects.example/x' } }) : new Response(null, { status: 404 });
	}
	if (dl && dl[1] === 'v9.9.8') throw new TypeError('fetch failed');                   // network down
	if (dl) return dl[1] === 'v9.9.9' ? new Response('Not Found', { status: 404 }) : new Response(IMAGE);
	throw new Error('unexpected fetch ' + u);
};

/* ---------- harness ---------- */
const ctx = { waitUntil() {} };
function mkEnv(over = {}) {
	return { ASSETS: { fetch: async () => new Response('static page') }, DB: new D1(), ACCESS_TEAM_DOMAIN: TEAM, ACCESS_AUD: AUD,
		ALLOWED_EMAIL_DOMAIN: 'siliconvandals.com', GITHUB_REPO: REPO, ...over };
}
async function call(env, method, path, { body, token, raw } = {}) {
	const headers = {};
	if (token !== null) headers['cf-access-jwt-assertion'] = token || await jwt();
	if (body !== undefined) headers['content-type'] = 'application/json';
	const res = await worker.fetch(new Request(`https://arribada.siliconvandals.com${path}`, { method, headers, body: body === undefined ? undefined : JSON.stringify(body) }), env, ctx);
	if (raw) return res;
	const text = await res.text();
	let data; try { data = JSON.parse(text); } catch (e) { data = text; }
	return { status: res.status, data, headers: res.headers };
}
const results = [];
function eq(a, b, msg) { if (JSON.stringify(a) !== JSON.stringify(b)) throw new Error(`${msg}: expected ${JSON.stringify(b)}, got ${JSON.stringify(a)}`); }
function ok(c, msg) { if (!c) throw new Error('assertion failed: ' + msg); }
async function test(name, fn) {
	try { await fn(); results.push([name, true]); console.log(`PASS  ${name}`); }
	catch (e) { results.push([name, false]); console.log(`FAIL  ${name}\n      ${e.stack || e}`); }
}

/* ---------- the browser's side: keys sealed with the team key (same format as teamSealKey in the page) ---------- */
const teamAes = await crypto.subtle.generateKey({ name: 'AES-GCM', length: 256 }, false, ['encrypt', 'decrypt']);
const b64 = u8 => Buffer.from(u8).toString('base64');
async function seal(id, hex, key = teamAes) {
	const iv = crypto.getRandomValues(new Uint8Array(12));
	const ct = new Uint8Array(await crypto.subtle.encrypt({ name: 'AES-GCM', iv, additionalData: new TextEncoder().encode(id) }, key, new TextEncoder().encode(hex)));
	return `v1:${b64(iv)}:${b64(ct)}`;
}
async function unseal(id, sealed, key = teamAes) {
	const [, iv, ct] = sealed.split(':');
	return new TextDecoder().decode(await crypto.subtle.decrypt({ name: 'AES-GCM', iv: Buffer.from(iv, 'base64'), additionalData: new TextEncoder().encode(id) }, key, Buffer.from(ct, 'base64')));
}
const crcOf = hex => (zcrc32(Buffer.from(hex, 'hex')) >>> 0).toString(16).toUpperCase().padStart(8, '0');
const KDF = { alg: 'PBKDF2-SHA256', iter: 600000, salt: b64(crypto.getRandomValues(new Uint8Array(16))), check: await seal('check', '0'.repeat(32)) };
async function sealed(list, key) {
	const out = [];
	for (const r of list) { const { key: k, ...rest } = r; out.push({ ...rest, key_enc: await seal(String(r.id), k.toUpperCase(), key), key_crc: crcOf(k) }); }
	return out;
}

// CLS-style sets (dummy values, not real credentials)
const SETS = [
	{ id: '100001', addr: '0a1b2c3d', key: '00112233445566778899aabbccddeeff', rconf: '' },
	{ id: '100002', addr: '0A1B2C3E', key: 'FFEEDDCCBBAA99887766554433221100' },
	{ id: '100003', addr: '0A1B2C3F', key: '0123456789ABCDEF0123456789ABCDEF', rconf: 'A'.repeat(32) },
];
async function seeded() {
	const env = mkEnv();
	eq((await call(env, 'POST', '/api/vault/setup', { body: { kdf: KDF } })).status, 200, 'setup');
	const r = await call(env, 'POST', '/api/vault/import', { body: { source: 'cls.csv', kdfVersion: 1, records: await sealed(SETS) } });
	eq([r.status, r.data.added.length], [200, 3], 'seed import');
	return env;
}

/* ---------- static + releases ---------- */
await test('non-API paths go to the static assets', async () => {
	const r = await call(mkEnv(), 'GET', '/index.html', { token: null });
	eq(r.data, 'static page', 'body');
});

await test('releases: Naiad app images only, no drafts or odd tags, notes per release', async () => {
	net.apiStatus = 200;
	const r = await call(mkEnv(), 'GET', '/api/releases', { token: null });
	eq(r.status, 200, 'status');
	eq(r.data.releases.map(x => x.tag), ['v1.0.1', 'v1.0.0'], 'tags');
	const v101 = r.data.releases[0];
	eq(v101.assets.map(a => `${a.variant}/${a.kind}`), ['release/app', 'bench/app', 'release/full'], 'app images + the full image for SWD');
	eq(v101.assets[0].url, '/api/firmware/v1.0.1/naiad-carrier-release__argos-smd-at-kineis-firmware.bin', 'proxied url');
	eq(v101.fwId, 'Naiad v1.0.1 (', 'fw id');
	eq(v101.warn, '', 'no warning');
	ok(/Do not deploy/.test(r.data.releases[1].warn), 'v1.0.0 flagged');
	eq(r.data.releases[1].fwId, '', 'v1.0.0 had the long id');
});

await test('releases: falls back to the Atom feed when the API is rate-limited', async () => {
	net.apiStatus = 403;
	const r = await call(mkEnv(), 'GET', '/api/releases', { token: null });
	net.apiStatus = 200;
	eq(r.status, 200, 'status');
	eq(r.data.releases.map(x => x.tag), ['v1.0.1', 'v1.0.0'], 'only tags with Naiad images, newest first');
	eq(r.data.releases.map(x => x.assets.filter(a => a.kind === 'app').map(a => a.variant).join('+')), ['release+bench', 'release'], 'images found');
	eq(r.data.releases[0].assets.filter(a => a.kind === 'full').map(a => a.name), ['naiad-carrier-release__argos-smd-at-kineis-firmware_full.bin', 'naiad-carrier-bench__argos-smd-at-kineis-firmware_full.bin'], 'full images listed with them');
	ok(/Do not deploy/.test(r.data.releases[1].warn), 'notes applied');
});

await test('firmware proxy: serves release images, refuses anything else', async () => {
	const env = mkEnv();
	const res = await call(env, 'GET', '/api/firmware/v1.0.1/naiad-carrier-release__argos-smd-at-kineis-firmware.bin', { token: null, raw: true });
	eq(res.status, 200, 'status');
	eq(res.headers.get('content-type'), 'application/octet-stream', 'type');
	eq(Buffer.from(await res.arrayBuffer()).equals(Buffer.from(IMAGE)), true, 'bytes');
	eq((await call(env, 'GET', '/api/firmware/v1.0.1/naiad-carrier-release__argos-smd-at-kineis-firmware_full.bin', { token: null })).status, 200, 'full image (SWD)');
	for (const p of ['/api/firmware/v1.0.1/naiad-carrier-release__argos-smd-at-kineis-firmware.elf', '/api/firmware/main/naiad-carrier-release__argos-smd-at-kineis-firmware.bin',
		'/api/firmware/v1.0.1/..%2F..%2Fsecret', '/api/firmware/v1.0.1/gui-pa-uart__argos-smd-at-kineis-firmware.bin']) {
		eq((await call(env, 'GET', p, { token: null })).status, 404, p);
	}
	eq((await call(env, 'GET', '/api/firmware/v9.9.9/naiad-carrier-release__argos-smd-at-kineis-firmware.bin', { token: null })).status, 404, 'missing on GitHub');
	const down = await call(env, 'GET', '/api/firmware/v9.9.8/naiad-carrier-release__argos-smd-at-kineis-firmware.bin', { token: null });
	eq([down.status, /cannot be reached/.test(down.data.error)], [502, true], 'GitHub unreachable: a clear 502, not an internal error');
	eq((await call(env, 'GET', '/api/nothing', { token: null })).status, 404, 'unknown api');
});

/* ---------- Access ---------- */
await test('vault is closed until Access and the database are configured', async () => {
	for (const [over, what] of [[{ ACCESS_AUD: '' }, 'no AUD'], [{ ACCESS_TEAM_DOMAIN: '' }, 'no team'], [{ ALLOWED_EMAIL_DOMAIN: '' }, 'no domain'], [{ DB: undefined }, 'no DB']]) {
		const r = await call(mkEnv(over), 'GET', '/api/vault/creds');
		eq([r.status, r.data.code], [503, 'not_configured'], what);
	}
});

await test('Access token checks: missing, forged, wrong audience/issuer/expiry/domain', async () => {
	const env = mkEnv(), now = Math.floor(Date.now() / 1000);
	const cases = [
		[null, 401, 'login', 'no token'],
		['abc.def', 401, 'login', 'malformed'],
		[await jwt({}, { key: rogueKey }), 401, 'login', 'forged signature'],
		[await jwt({}, { header: { kid: 'other' } }), 401, 'login', 'unknown kid'],
		[await jwt({}, { header: { alg: 'none' } }), 401, 'login', 'alg none'],
		[await jwt({ aud: ['someone-else'] }), 401, 'login', 'audience'],
		[await jwt({ iss: 'https://evil.cloudflareaccess.com' }), 401, 'login', 'issuer'],
		[await jwt({ exp: now - 120 }), 401, 'login', 'expired'],
		[await jwt({ nbf: now + 600 }), 401, 'login', 'not yet valid'],
		[await jwt({ email: 'tom@gmail.com' }), 403, 'forbidden', 'other domain'],
		[await jwt({ email: 'tom@siliconvandals.com.evil.io' }), 403, 'forbidden', 'domain suffix trick'],
	];
	for (const [token, status, code, what] of cases) {
		const r = await call(env, 'GET', '/api/vault/me', { token });
		eq([r.status, r.data.code], [status, code], what);
	}
	const me = await call(env, 'GET', '/api/vault/me', { token: await jwt({ email: 'Tom@SiliconVandals.com' }) });
	eq([me.status, me.data.email, me.data.kdf], [200, 'tom@siliconvandals.com', null], 'valid token, no vault yet');
});

await test('login page reports the account to the tool and closes', async () => {
	const res = await call(mkEnv(), 'GET', '/api/vault/login', { raw: true });
	eq(res.status, 200, 'status');
	ok(/default-src 'none'/.test(res.headers.get('content-security-policy')), 'csp');
	const html = await res.text();
	ok(html.includes('naiadLogin:"tom@siliconvandals.com"') && html.includes("BroadcastChannel('naiad-login')"), 'posts the login');
});

/* ---------- vault ---------- */
await test('setup: the first user sets the passphrase parameters once; nothing works before', async () => {
	const env = mkEnv();
	const before = await call(env, 'POST', '/api/vault/import', { body: { kdfVersion: 1, records: await sealed(SETS) } });
	eq([before.status, before.data.code], [409, 'no_vault'], 'no vault yet');
	for (const bad of [{ ...KDF, iter: 1000 }, { ...KDF, salt: 'short' }, { ...KDF, check: 'plain' }, { ...KDF, alg: 'MD5' }]) {
		eq((await call(env, 'POST', '/api/vault/setup', { body: { kdf: bad } })).status, 400, JSON.stringify(bad).slice(0, 40));
	}
	const s = await call(env, 'POST', '/api/vault/setup', { body: { kdf: KDF } });
	eq([s.status, s.data.kdf.salt, s.data.kdf.check, s.data.kdf.version, s.data.kdf.by], [200, KDF.salt, KDF.check, 1, 'tom@siliconvandals.com'], 'created');
	const again = await call(env, 'POST', '/api/vault/setup', { body: { kdf: { ...KDF, salt: b64(new Uint8Array(16)) } }, token: await jwt({ email: 'lucas@siliconvandals.com' }) });
	eq([again.status, again.data.kdf.salt], [409, KDF.salt], 'second setup refused, first kept');
	eq((await call(env, 'GET', '/api/vault/me')).data.kdf.salt, KDF.salt, 'me returns it');
	eq((await call(env, 'GET', '/api/vault/history')).data.history.map(h => `${h.ev} ${h.by}`), ['vault created tom@siliconvandals.com'], 'logged once');
});

await test('import: only sealed keys accepted; identical / conflicting / invalid sets reported', async () => {
	const env = await seeded();
	const rows = env.DB.db.prepare('SELECT * FROM creds ORDER BY id').all();
	eq(rows.map(r => r.id), ['100001', '100002', '100003'], 'ids');
	eq(rows[0].addr, '0A1B2C3D', 'addr normalised');
	for (const [i, r] of rows.entries()) {
		eq(await unseal(r.id, r.key_enc), SETS[i].key.toUpperCase(), 'stored ciphertext opens in the browser');
		eq(r.key_crc, crcOf(SETS[i].key), 'key CRC = unit +SECKEYCRC');
		eq(r.updated_by, 'tom@siliconvandals.com', 'who');
	}
	const again = await call(env, 'POST', '/api/vault/import', { body: { source: 'x', kdfVersion: 1, records: [
		...(await sealed([SETS[0], { ...SETS[1], key: '1'.repeat(32) }, { id: '100009', addr: SETS[2].addr, key: '2'.repeat(32) }, { id: '100010', addr: 'ZZ', key: '3'.repeat(32) }])),
		...(await sealed([{ id: '100011', addr: '0B000001', key: '4'.repeat(32), status: 'in use', sn: 'SMD1', note: 'from a local vault' }, { id: '100012', addr: '0B000001', key: '5'.repeat(32) }])),
		{ id: '100013', addr: '0B000002', key: '6'.repeat(32) },                      // a clear key is refused
		{ id: '100014', addr: '0B000003', key_enc: await seal('100014', '7'.repeat(32)) },  // no CRC
	] } });
	eq(again.data.added, ['100011'], 'added');
	eq(again.data.identical, ['100001'], 'identical');
	eq(again.data.conflicts.map(c => c.id + ': ' + c.why), ['100002: KEY differs from the vault', `100009: ADDR ${SETS[2].addr} already belongs to ID 100003`, '100012: ADDR 0B000001 already belongs to ID 100011'], 'conflicts');
	eq(again.data.invalid.map(x => x.id + ':' + Object.keys(x.errors)), ['100010:addr', '100013:key_enc,key_crc', '100014:key_crc'], 'invalid');
	ok(!JSON.stringify(env.DB.db.prepare('SELECT * FROM creds').all()).includes('6'.repeat(32)), 'clear key never stored');
	const rec = env.DB.db.prepare("SELECT status, sn, note FROM creds WHERE id = '100011'").get();
	eq([rec.status, rec.sn, rec.note], ['in use', 'SMD1', 'from a local vault'], 'tracking fields kept');
	eq((await call(env, 'POST', '/api/vault/import', { body: { kdfVersion: 1, records: new Array(501).fill(SETS[0]) } })).status, 413, 'size cap');
	eq((await call(env, 'POST', '/api/vault/import', { body: { kdfVersion: 7, records: [] } })).data.code, 'rekeyed', 'stale passphrase version');
});

await test('list carries no keys; a key fetch returns ciphertext and is logged', async () => {
	const env = await seeded();
	const l = await call(env, 'GET', '/api/vault/creds');
	eq([l.data.records.length, l.data.kdfVersion], [3, 1], 'count');
	ok(l.data.records.every(r => !('key' in r) && !('key_enc' in r)), 'no keys in the list');
	const k = await call(env, 'GET', '/api/vault/creds/100002/key?why=provision', { token: await jwt({ email: 'lucas@siliconvandals.com' }) });
	eq(await unseal('100002', k.data.key_enc), SETS[1].key, 'opens in the browser');
	const h = await call(env, 'GET', '/api/vault/creds/100002/history');
	eq(h.data.history.map(x => [x.ev, x.by, x.note]), [['imported', 'tom@siliconvandals.com', 'cls.csv'], ['key read', 'lucas@siliconvandals.com', 'provision']], 'history');
	eq((await call(env, 'GET', '/api/vault/creds/999/key')).status, 404, 'unknown id');
});

await test('tracking: provisioned on a unit, then erased', async () => {
	const env = await seeded();
	let r = await call(env, 'POST', '/api/vault/creds/100001/track', { body: { kind: 'provisioned', sn: 'SMD7130029002F', fw: 'Naiad v1.0.1 (abc1234)' } });
	eq([r.data.record.status, r.data.record.sn, r.data.record.fw, r.data.record.version], ['in use', 'SMD7130029002F', 'Naiad v1.0.1 (abc1234)', 2], 'in use');
	r = await call(env, 'POST', '/api/vault/creds/100001/track', { body: { kind: 'provisioned', sn: 'SMD2' } });
	r = await call(env, 'POST', '/api/vault/creds/100001/track', { body: { kind: 'erased', sn: 'SMD2', t: '2026-01-01T10:00:00Z' } });
	eq([r.data.record.status, r.data.record.sn], ['free', ''], 'free again');
	const h = await call(env, 'GET', '/api/vault/creds/100001/history');
	eq(h.data.history.map(x => `${x.ev}: ${x.note}`), ['imported: cls.csv', 'provisioned: SMD7130029002F', 'provisioned: SMD2 (was on SMD7130029002F)', 'erased: from SMD2 (recorded 2026-01-01 10:00 UTC)'], 'history');
	eq((await call(env, 'POST', '/api/vault/creds/100001/track', { body: { kind: 'stolen' } })).status, 400, 'bad kind');
	eq((await call(env, 'POST', '/api/vault/creds/123/track', { body: { kind: 'erased' } })).data.code, 'unknown_id', 'unknown id');
});

await test('edit: version check, validation, ADDR uniqueness, new key sealed and logged by CRC only', async () => {
	const env = await seeded();
	const v1 = (await call(env, 'GET', '/api/vault/creds')).data.records[0];
	const patch = b => call(env, 'PATCH', '/api/vault/creds/100001', { body: b });
	eq((await patch({ version: v1.version + 1, note: 'x' })).status, 409, 'stale version');
	eq([(await patch({ version: 1, addr: 'xyz' })).data.error], ['ADDR: 8 hex'], 'bad addr');
	eq((await patch({ version: 1, status: 'lost' })).status, 400, 'bad status');
	eq((await patch({ version: 1, addr: SETS[1].addr })).status, 409, 'addr taken');
	let r = await patch({ version: 1, status: 'retired', note: 'antenna damaged', id: '5' });
	eq([r.status, r.data.record.status, r.data.record.note, r.data.record.id, r.data.record.version], [200, 'retired', 'antenna damaged', '100001', 2], 'edited');
	const newKey = 'ABCDEF0123456789ABCDEF0123456789';
	eq((await patch({ version: 2, key_enc: 'v1:plain' , key_crc: crcOf(newKey), kdfVersion: 1 })).status, 400, 'unsealed key refused');
	eq((await patch({ version: 2, key_enc: await seal('100001', newKey), kdfVersion: 1 })).status, 400, 'CRC required with a new key');
	eq((await patch({ version: 2, key_enc: await seal('100001', newKey), key_crc: crcOf(newKey), kdfVersion: 0 })).data.code, 'rekeyed', 'stale passphrase version');
	r = await patch({ version: 2, key_enc: await seal('100001', newKey), key_crc: crcOf(newKey), kdfVersion: 1 });
	eq(r.data.record.key_crc, crcOf(newKey), 'new CRC');
	eq(await unseal('100001', (await call(env, 'GET', '/api/vault/creds/100001/key')).data.key_enc), newKey, 'new key stored sealed');
	eq((await patch({ version: 3, note: 'antenna damaged' })).data.record.version, 3, 'no-op edit keeps the version');
	const h = (await call(env, 'GET', '/api/vault/creds/100001/history')).data.history;
	eq(h.map(x => `${x.ev}|${x.note}`), ['imported|cls.csv', 'status: free → retired|note', `edited|key (CRC-32 ${crcOf(newKey)})`, 'key read|'], 'history');
});

await test('concurrent edit: the losing write changes nothing and logs nothing', async () => {
	const env = await seeded();
	const [a, b] = await Promise.all([
		call(env, 'PATCH', '/api/vault/creds/100002', { body: { version: 1, note: 'first' } }),
		call(env, 'PATCH', '/api/vault/creds/100002', { body: { version: 1, note: 'second' } }),
	]);
	eq([a.status, b.status].sort(), [200, 409], 'one wins');
	eq((await call(env, 'GET', '/api/vault/creds/100002/history')).data.history.length, 2, 'one edit logged');
});

await test('delete needs the current version; history survives', async () => {
	const env = await seeded();
	eq((await call(env, 'DELETE', '/api/vault/creds/100003?version=7')).status, 409, 'stale');
	eq((await call(env, 'DELETE', '/api/vault/creds/100003?version=1')).status, 200, 'deleted');
	eq((await call(env, 'GET', '/api/vault/creds')).data.records.map(r => r.id), ['100001', '100002'], 'remaining');
	eq(env.DB.db.prepare("SELECT ev FROM history WHERE cred_id = '100003' ORDER BY seq").all().map(x => x.ev), ['imported', 'deleted'], 'history kept');
});

await test('export (offline copy) returns every sealed key and logs it per set', async () => {
	const env = await seeded();
	const r = await call(env, 'GET', '/api/vault/export?why=offline%20copy');
	eq(r.data.kdf.salt, KDF.salt, 'kdf for the copy');
	for (const [i, x] of r.data.records.entries()) eq(await unseal(x.id, x.key_enc), SETS[i].key.toUpperCase(), 'opens');
	eq(env.DB.db.prepare("SELECT COUNT(*) AS n FROM history WHERE ev = 'key exported' AND note = 'offline copy'").get().n, 3, 'logged');
});

await test('passphrase change: every key re-sealed atomically; stale or partial requests refused', async () => {
	const env = await seeded();
	const ex = (await call(env, 'GET', '/api/vault/export')).data;
	const newAes = await crypto.subtle.generateKey({ name: 'AES-GCM', length: 256 }, false, ['encrypt', 'decrypt']);
	const kdf2 = { ...KDF, salt: b64(crypto.getRandomValues(new Uint8Array(16))), check: await seal('check', '0'.repeat(32), newAes) };
	const keys = [];
	for (const r of ex.records) keys.push({ id: r.id, version: r.version, key_enc: await seal(r.id, await unseal(r.id, r.key_enc), newAes) });
	const rekey = b => call(env, 'POST', '/api/vault/rekey', { body: { version: 1, kdf: kdf2, keys, ...b } });
	eq((await rekey({ version: 2 })).status, 409, 'stale vault version');
	eq((await rekey({ keys: keys.slice(1) })).status, 409, 'a set missing');
	eq((await rekey({ keys: [...keys.slice(1), { ...keys[0], version: 9 }] })).status, 409, 'a set changed meanwhile');
	eq((await rekey({ keys: [...keys.slice(1), { ...keys[0], key_enc: 'clear' }] })).status, 400, 'unsealed key');
	const r = await rekey({});
	eq([r.status, r.data.kdf.version, r.data.kdf.salt], [200, 2, kdf2.salt], 'rekeyed');
	for (const [i, row] of env.DB.db.prepare('SELECT id, key_enc, version FROM creds ORDER BY id').all().entries()) {
		eq(await unseal(row.id, row.key_enc, newAes), SETS[i].key.toUpperCase(), 'opens with the new key');
		eq(row.version, 2, 'record versions bumped (pending edits must reload)');
	}
	eq((await call(env, 'GET', '/api/vault/creds')).data.kdfVersion, 2, 'list tells stations');
	eq((await call(env, 'POST', '/api/vault/import', { body: { kdfVersion: 1, records: await sealed([{ id: '100020', addr: '0C000001', key: '8'.repeat(32) }]) } })).data.code, 'rekeyed', 'old-key import refused');
	eq((await call(env, 'GET', '/api/vault/history')).data.history.map(h => h.ev), ['vault created', 'passphrase changed'], 'vault history');
});

const failed = results.filter(r => !r[1]).length;
console.log(`\n${results.length - failed}/${results.length} passed`);
process.exit(failed ? 1 : 0);
