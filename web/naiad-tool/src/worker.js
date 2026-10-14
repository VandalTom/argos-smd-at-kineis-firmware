/**
 * Naiad tool Worker: the static page (dist/, served before this code runs) plus a small API.
 *
 *   GET  /api/releases                    Naiad firmware releases on GitHub (cached 5 min)
 *   GET  /api/firmware/<tag>/<file>       one release image, proxied same-origin (cached 1 hour)
 *   /api/vault/...                        the team's shared credentials vault (D1)
 *
 * The vault takes two keys. Cloudflare Access (Google, @siliconvandals.com) decides who reaches
 * it: Access lets only signed-in users through and adds a signed JWT, and every vault request
 * re-verifies that JWT here (signature against the team's keys, audience, issuer, expiry, email
 * domain), so a request that reaches the Worker any other way (workers.dev, an Access app removed
 * by mistake) is refused. Until Access and the database are set up, the vault answers 503.
 *
 * The team passphrase protects the SECKEYs: the page encrypts each key in the browser (AES-256-GCM,
 * key derived from the passphrase with PBKDF2, the set's ID as associated data) and this Worker
 * only stores and returns that ciphertext, next to the key's CRC-32 for comparisons. IDs, ADDRs
 * and tracking fields are plain, so units are tracked without the passphrase. Every key fetch,
 * export, import, edit and deletion is logged with the user's email.
 */


/** App image (UART bootloader updates) or '_full' = app + bootloader from 0x08000000 (SWD flashing). */
const RELEASE_ASSET = /^naiad-carrier-(release|bench)__argos-smd-at-kineis-firmware(_full)?\.bin$/;
const assetInfo = name => { const m = RELEASE_ASSET.exec(name); return { variant: m[1], kind: m[2] ? 'full' : 'app' }; };
const VARIANTS = ['release', 'bench'];
const TAG = /^v\d+\.\d+\.\d+$/;
/** Per-release notes shown in the tool. fwId: false = predates the short 'Naiad vX.Y.Z (sha)' ID. */
const GPS_PORT_WARN = 'Do not deploy: the GPS port loses its data whenever the processor idles, so position fixes fail; the WS85 wind sensor shares that port (fixed in v1.0.3).';
const RELEASE_NOTES = {
	'v1.0.0': { warn: 'Do not deploy: units stop transmitting after their first deep sleep (fixed in v1.0.1).', fwId: false },
	'v1.0.1': { warn: GPS_PORT_WARN },
	'v1.0.2': { warn: GPS_PORT_WARN },
	'v1.0.3': { warn: 'Do not deploy: on the carrier the unit reads no WS85 wind or rain data (each WS85 frame loses its first bytes); fixed in v1.0.4.' },
};
const STATUSES = ['free', 'in use', 'retired'];
const EDITABLE = ['addr', 'key_enc', 'rconf', 'status', 'sn', 'fw', 'assigned', 'note'];
const IMPORT_MAX = 500;

class HttpError extends Error { constructor(status, msg, extra) { super(msg); this.status = status; this.extra = extra; } }

const json = (data, status = 200, headers = {}) => new Response(JSON.stringify(data), {
	status, headers: { 'content-type': 'application/json; charset=utf-8', 'cache-control': 'no-store', 'x-content-type-options': 'nosniff', ...headers },
});

export default {
	async fetch(request, env, ctx) {
		const url = new URL(request.url);
		if (!url.pathname.startsWith('/api/')) return env.ASSETS.fetch(request);
		try {
			if (url.pathname === '/api/releases' && request.method === 'GET') return await releases(env, ctx);
			const fw = url.pathname.match(/^\/api\/firmware\/([^/]+)\/([^/]+)$/);
			if (fw && request.method === 'GET') return await firmware(env, ctx, decodeURIComponent(fw[1]), decodeURIComponent(fw[2]));
			if (url.pathname === '/api/vault' || url.pathname.startsWith('/api/vault/')) return await vault(request, env, url);
			throw new HttpError(404, 'Not found');
		} catch (e) {
			if (e instanceof HttpError) return json({ error: e.message, ...(e.extra || {}) }, e.status);
			console.error(e && e.stack || e);
			return json({ error: 'Internal error' }, 500);
		}
	},
};

/* ---------------- firmware releases ---------------- */

/** fetch() to GitHub: a network failure becomes a clear 502 instead of an internal error. */
async function gh(url, init) {
	try { return await fetch(url, init); }
	catch (e) { throw new HttpError(502, 'GitHub cannot be reached right now: try again in a minute'); }
}

async function cached(ctx, key, ttl, make) {
	const cache = globalThis.caches && caches.default;
	const req = new Request(`https://naiad-tool.cache/${key}`);
	const hit = cache && await cache.match(req);
	if (hit) return hit;
	const res = await make();
	if (cache && res.ok) {
		const copy = new Response(res.clone().body, res);
		copy.headers.set('cache-control', `public, max-age=${ttl}`);
		ctx.waitUntil(cache.put(req, copy));
	}
	return res;
}

/** Expected AT+FW=? text for a release ('Naiad vX.Y.Z (' from v1.0.1 on). */
function fwIdFor(tag) {
	const note = RELEASE_NOTES[tag];
	return note && note.fwId === false ? '' : `Naiad ${tag} (`;
}

function releaseEntry(env, tag, { name, date, prerelease = false, url, assets }) {
	return {
		tag, name: name || tag, date: date || '', prerelease, url: url || `https://github.com/${env.GITHUB_REPO}/releases/tag/${tag}`,
		warn: (RELEASE_NOTES[tag] || {}).warn || '', fwId: fwIdFor(tag),
		assets: assets.map(a => ({ ...a, url: `/api/firmware/${encodeURIComponent(tag)}/${encodeURIComponent(a.name)}` })),
	};
}

const semver = t => t.slice(1).split('.').map(Number);
const newestFirst = (a, b) => { const x = semver(a.tag), y = semver(b.tag); return y[0] - x[0] || y[1] - x[1] || y[2] - x[2]; };
const FEED_MAX = 10;

/** The REST API (sizes, drafts, prereleases); when GitHub rate-limits the shared Worker IPs (usual on
 *  Cloudflare without a GITHUB_TOKEN), the public Atom feed stands in. The feed also lists plain tags
 *  (the fork inherits upstream's), so each tag counts only if its Naiad images are really attached. */
async function listReleases(env) {
	const auth = { 'user-agent': 'naiad-tool', ...(env.GITHUB_TOKEN ? { authorization: `Bearer ${env.GITHUB_TOKEN}` } : {}) };
	const api = await fetch(`https://api.github.com/repos/${env.GITHUB_REPO}/releases?per_page=30`, { headers: { ...auth, accept: 'application/vnd.github+json' } }).catch(() => ({ ok: false, status: 0 }));
	if (api.ok) {
		return (await api.json()).filter(r => !r.draft && TAG.test(r.tag_name)).map(r => releaseEntry(env, r.tag_name, {
			name: r.name, date: r.published_at, prerelease: !!r.prerelease, url: r.html_url,
			assets: r.assets.filter(a => RELEASE_ASSET.test(a.name)).map(a => ({ name: a.name, size: a.size, ...assetInfo(a.name) })),
		})).filter(r => r.assets.some(a => a.kind === 'app')).sort(newestFirst);
	}
	console.log(`GitHub API ${api.status}: listing releases from the Atom feed`);
	const feed = await gh(`https://github.com/${env.GITHUB_REPO}/releases.atom`, { headers: { 'user-agent': 'naiad-tool' } });
	if (!feed.ok) throw new HttpError(502, `GitHub answered ${api.status} (API) and ${feed.status} (feed)`);
	const tags = [];
	for (const entry of (await feed.text()).match(/<entry>[\s\S]*?<\/entry>/g) || []) {
		const tag = decodeURIComponent((entry.match(/\/releases\/tag\/([^"<]+)"/) || [])[1] || '');
		if (TAG.test(tag) && !tags.some(t => t.tag === tag)) tags.push({ tag, date: (entry.match(/<updated>([^<]+)<\/updated>/) || [])[1] });
	}
	tags.sort(newestFirst);
	const found = await Promise.all(tags.slice(0, FEED_MAX).map(async t => {
		const assets = [];
		for (const variant of VARIANTS) {
			const name = `naiad-carrier-${variant}__argos-smd-at-kineis-firmware.bin`;
			const head = await gh(`https://github.com/${env.GITHUB_REPO}/releases/download/${t.tag}/${name}`, { method: 'HEAD', redirect: 'manual', headers: { 'user-agent': 'naiad-tool' } });
			// CI attaches the '_full' image next to every app image (one HEAD per variant keeps the subrequest count low)
			if (head.status === 302 || head.status === 200) assets.push({ name, size: null, variant, kind: 'app' }, { name: name.replace(/\.bin$/, '_full.bin'), size: null, variant, kind: 'full' });
		}
		return assets.length ? releaseEntry(env, t.tag, { date: t.date, assets }) : null;
	}));
	return found.filter(Boolean);
}

async function releases(env, ctx) {
	const res = await cached(ctx, 'releases-v3', 300, async () => json({ repo: env.GITHUB_REPO, releases: await listReleases(env) }));
	return new Response(res.body, { status: res.status, headers: { 'content-type': 'application/json; charset=utf-8', 'cache-control': 'public, max-age=120', 'x-content-type-options': 'nosniff' } });
}

async function firmware(env, ctx, tag, name) {
	if (!TAG.test(tag) || !RELEASE_ASSET.test(name)) throw new HttpError(404, 'Not a Naiad release image');
	return cached(ctx, `fw2/${tag}/${name}`, 3600, async () => {     // 1 h: a re-published release reaches the tool quickly
		const res = await gh(`https://github.com/${env.GITHUB_REPO}/releases/download/${tag}/${name}`, { redirect: 'follow', headers: { 'user-agent': 'naiad-tool' } });
		if (!res.ok) throw new HttpError(res.status === 404 ? 404 : 502, `Release image not available (GitHub ${res.status})`);
		return new Response(res.body, {
			headers: { 'content-type': 'application/octet-stream', 'content-disposition': `attachment; filename="${tag}__${name}"`, 'x-content-type-options': 'nosniff' },
		});
	});
}

/* ---------------- Cloudflare Access verification ---------------- */

const b64url = s => Uint8Array.from(atob(s.replace(/-/g, '+').replace(/_/g, '/') + '==='.slice((s.length + 3) % 4)), c => c.charCodeAt(0));
let jwksCache = { team: '', at: 0, keys: [] };

async function accessKeys(team, force) {
	if (!force && jwksCache.team === team && Date.now() - jwksCache.at < 3600e3) return jwksCache.keys;
	const res = await fetch(`https://${team}/cdn-cgi/access/certs`);
	if (!res.ok) throw new HttpError(503, 'Cannot fetch the Cloudflare Access signing keys');
	jwksCache = { team, at: Date.now(), keys: (await res.json()).keys || [] };
	return jwksCache.keys;
}

/** -> signed-in email (lower case); throws 401 (login), 403 (wrong account) or 503 (not set up). */
async function verifyAccess(request, env) {
	const team = env.ACCESS_TEAM_DOMAIN, aud = env.ACCESS_AUD, domain = String(env.ALLOWED_EMAIL_DOMAIN || '').toLowerCase();
	if (!team || !aud || !domain) throw new HttpError(503, 'The team vault is not set up yet (Cloudflare Access)', { code: 'not_configured' });
	const login = msg => new HttpError(401, msg, { code: 'login' });
	const token = request.headers.get('cf-access-jwt-assertion');
	if (!token) throw login(`Sign in with your @${domain} account`);
	const parts = token.split('.');
	if (parts.length !== 3) throw login('Malformed login token');
	let header, payload;
	try { header = JSON.parse(new TextDecoder().decode(b64url(parts[0]))); payload = JSON.parse(new TextDecoder().decode(b64url(parts[1]))); }
	catch (e) { throw login('Malformed login token'); }
	if (header.alg !== 'RS256') throw login('Unexpected login token algorithm');
	let jwk = (await accessKeys(team)).find(k => k.kid === header.kid);
	if (!jwk) jwk = (await accessKeys(team, true)).find(k => k.kid === header.kid);   // keys rotated
	if (!jwk) throw login('Unknown login token key');
	const key = await crypto.subtle.importKey('jwk', { kty: jwk.kty, n: jwk.n, e: jwk.e }, { name: 'RSASSA-PKCS1-v1_5', hash: 'SHA-256' }, false, ['verify']);
	if (!(await crypto.subtle.verify('RSASSA-PKCS1-v1_5', key, b64url(parts[2]), new TextEncoder().encode(`${parts[0]}.${parts[1]}`)))) throw login('Login token signature invalid');
	const t = Math.floor(Date.now() / 1000);
	if (!(Array.isArray(payload.aud) ? payload.aud : [payload.aud]).includes(aud)) throw login('Login token is for another application');
	if (payload.iss !== `https://${team}`) throw login('Login token from another issuer');
	if (typeof payload.exp !== 'number' || payload.exp < t - 30) throw login('Login expired: sign in again');
	if (typeof payload.nbf === 'number' && payload.nbf > t + 30) throw login('Login token not valid yet');
	const email = String(payload.email || '').toLowerCase();
	if (!email.endsWith('@' + domain)) throw new HttpError(403, `Only @${domain} accounts can use the team vault`, { code: 'forbidden' });
	return email;
}


/* ---------------- validation ---------------- */

/** A key sealed in the browser: 'v1:' + base64 12-byte IV + ':' + base64 (32 hex chars + 16-byte GCM tag). */
const isSealed = s => typeof s === 'string' && /^v1:[A-Za-z0-9+/]{16}:[A-Za-z0-9+/]{64}$/.test(s);
const isHex = (s, n) => typeof s === 'string' && s.length === n && /^[0-9A-F]*$/.test(s);
function normalize(src) {
	const clean = s => String(s == null ? '' : s).trim().replace(/[\s:-]+/g, '').replace(/^0x/i, '').toUpperCase();
	const status = String(src.status == null ? '' : src.status).trim().toLowerCase();
	const r = {
		id: String(src.id == null ? '' : src.id).trim(), addr: clean(src.addr), key_enc: String(src.key_enc || ''), key_crc: clean(src.key_crc),
		rconf: clean(src.rconf), status: status || 'free', sn: String(src.sn || '').trim().slice(0, 64), fw: String(src.fw || '').trim().slice(0, 96),
		assigned: String(src.assigned || '').trim().slice(0, 10), note: String(src.note || '').trim().slice(0, 500),
	};
	const errors = {};
	if (!/^\d+$/.test(r.id) || Number(r.id) < 1 || Number(r.id) > 2 ** 31) errors.id = 'decimal 1..2147483648'; else r.id = String(Number(r.id));
	if (!isHex(r.addr, 8)) errors.addr = '8 hex';
	if (!isSealed(r.key_enc)) errors.key_enc = 'key sealed with the team passphrase';
	if (!isHex(r.key_crc, 8)) errors.key_crc = '8 hex';
	if (r.rconf && !isHex(r.rconf, 32)) errors.rconf = '32 hex or empty';
	if (!STATUSES.includes(r.status)) errors.status = STATUSES.join(', ');
	if (r.assigned && !/^\d{4}-\d{2}-\d{2}$/.test(r.assigned)) errors.assigned = 'YYYY-MM-DD or empty';
	return { rec: r, errors };
}
/** Key-derivation parameters + passphrase check, created by the first user (see the page's teamVaultNew). */
function validKdf(k) {
	return k && k.alg === 'PBKDF2-SHA256' && Number.isInteger(k.iter) && k.iter >= 100000 && k.iter <= 10000000
		&& typeof k.salt === 'string' && /^[A-Za-z0-9+/]{22}==$/.test(k.salt) && isSealed(k.check);
}

/* ---------------- vault (D1) ---------------- */

const SCHEMA = [
	`CREATE TABLE IF NOT EXISTS meta (
		k TEXT PRIMARY KEY, v TEXT NOT NULL, version INTEGER NOT NULL DEFAULT 1, updated TEXT NOT NULL, updated_by TEXT NOT NULL)`,
	`CREATE TABLE IF NOT EXISTS creds (
		id TEXT PRIMARY KEY, addr TEXT NOT NULL UNIQUE, key_enc TEXT NOT NULL, key_crc TEXT NOT NULL,
		rconf TEXT NOT NULL DEFAULT '', status TEXT NOT NULL DEFAULT 'free', sn TEXT NOT NULL DEFAULT '',
		fw TEXT NOT NULL DEFAULT '', assigned TEXT NOT NULL DEFAULT '', note TEXT NOT NULL DEFAULT '',
		source TEXT NOT NULL DEFAULT '', added TEXT NOT NULL, updated TEXT NOT NULL,
		updated_by TEXT NOT NULL DEFAULT '', version INTEGER NOT NULL DEFAULT 1)`,
	`CREATE TABLE IF NOT EXISTS history (
		seq INTEGER PRIMARY KEY AUTOINCREMENT, cred_id TEXT NOT NULL, t TEXT NOT NULL, ev TEXT NOT NULL,
		by TEXT NOT NULL DEFAULT '', note TEXT NOT NULL DEFAULT '')`,
	`CREATE INDEX IF NOT EXISTS history_cred ON history(cred_id, seq)`,
];
const schemaReady = new WeakMap();
function schema(env) {
	if (!env.DB) throw new HttpError(503, 'No D1 database is bound to the Worker', { code: 'not_configured' });
	if (!schemaReady.has(env.DB)) schemaReady.set(env.DB, env.DB.batch(SCHEMA.map(s => env.DB.prepare(s))).catch(e => { schemaReady.delete(env.DB); throw e; }));
	return schemaReady.get(env.DB);
}

const PUBLIC_COLS = 'id, addr, key_crc, rconf, status, sn, fw, assigned, note, source, added, updated, updated_by, version';
const VAULT_ROW = '*';                            // history rows about the vault itself
const getRec = (env, id) => env.DB.prepare(`SELECT ${PUBLIC_COLS} FROM creds WHERE id = ?`).bind(id).first();
async function getKdf(env) {
	const row = await env.DB.prepare("SELECT v, version, updated, updated_by FROM meta WHERE k = 'kdf'").first();
	return row ? { ...JSON.parse(row.v), version: row.version, created: row.updated, by: row.updated_by } : null;
}
const hist = (env, id, ev, by, note = '') => env.DB.prepare('INSERT INTO history (cred_id, t, ev, by, note) VALUES (?, ?, ?, ?, ?)').bind(id, new Date().toISOString(), ev, by, note);
/** One history row per ID in a single statement (D1 caps queries per request, not rows). */
const histMany = (env, ids, ev, by, note = '') => env.DB.prepare(
	'INSERT INTO history (cred_id, t, ev, by, note) SELECT value, ?, ?, ?, ? FROM json_each(?)').bind(new Date().toISOString(), ev, by, note, JSON.stringify(ids));
const changedBy = (id, cur) => new HttpError(409, `ID ${id} was changed by ${cur.updated_by || 'someone else'}: reloaded, try again`, { record: cur });

async function body(request) {
	try { return await request.json(); } catch (e) { throw new HttpError(400, 'Expected a JSON body'); }
}

const LOGIN_PAGE = email => `<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Signed in</title><body style="font:15px system-ui,sans-serif;padding:24px">Signed in as <b>${email}</b>. You can close this window.
<script>(function(){var m={naiadLogin:"${email}"};try{new BroadcastChannel('naiad-login').postMessage(m)}catch(e){}
try{if(window.opener)window.opener.postMessage(m,location.origin)}catch(e){}setTimeout(function(){window.close()},800)})()</script>`;

async function vault(request, env, url) {
	const email = await verifyAccess(request, env);
	const path = url.pathname.replace(/^\/api\/vault\/?/, ''), method = request.method;
	if (path === 'login' && method === 'GET') {
		const safe = email.replace(/[^a-z0-9@._+-]/g, '');
		return new Response(LOGIN_PAGE(safe), { headers: { 'content-type': 'text/html; charset=utf-8', 'cache-control': 'no-store', 'content-security-policy': "default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; frame-ancestors 'none'" } });
	}
	await schema(env);
	if (path === 'me' && method === 'GET') return json({ email, kdf: await getKdf(env) });

	if (path === 'setup' && method === 'POST') {                  // first user picks the team passphrase
		const b = await body(request);
		if (!validKdf(b.kdf)) throw new HttpError(400, 'Invalid key-derivation parameters');
		const now = new Date().toISOString();
		const v = JSON.stringify({ alg: b.kdf.alg, iter: b.kdf.iter, salt: b.kdf.salt, check: b.kdf.check });
		const [ins] = await env.DB.batch([
			env.DB.prepare("INSERT OR IGNORE INTO meta (k, v, updated, updated_by) VALUES ('kdf', ?, ?, ?)").bind(v, now, email),
			env.DB.prepare('INSERT INTO history (cred_id, t, ev, by, note) SELECT ?, ?, ?, ?, ? WHERE changes() > 0').bind(VAULT_ROW, now, 'vault created', email, ''),
		]);
		if (!ins.meta || ins.meta.changes === 0) throw new HttpError(409, 'The team vault already exists: unlock it with the team passphrase', { kdf: await getKdf(env) });
		return json({ kdf: await getKdf(env) });
	}

	const kdf = await getKdf(env);
	if (!kdf) throw new HttpError(409, 'No team vault yet: create it with the team passphrase', { code: 'no_vault' });

	if (path === 'rekey' && method === 'POST') {                   // new passphrase: every key re-sealed in the browser
		const b = await body(request);
		if (!validKdf(b.kdf)) throw new HttpError(400, 'Invalid key-derivation parameters');
		if (Number(b.version) !== kdf.version) throw new HttpError(409, 'The passphrase was changed meanwhile: unlock again');
		const keys = Array.isArray(b.keys) ? b.keys : [];
		if (!keys.every(k => k && isSealed(k.key_enc))) throw new HttpError(400, 'Every key must be sealed with the new passphrase');
		const { results: cur } = await env.DB.prepare('SELECT id, version FROM creds').all();
		const sent = new Map(keys.map(k => [String(k.id), Number(k.version)]));
		if (sent.size !== cur.length || keys.length !== cur.length || cur.some(r => sent.get(r.id) !== r.version)) throw new HttpError(409, 'Sets were added or changed meanwhile: unlock again and retry');
		const now = new Date().toISOString();
		await env.DB.batch([
			env.DB.prepare("UPDATE meta SET v = ?, version = version + 1, updated = ?, updated_by = ? WHERE k = 'kdf'")
				.bind(JSON.stringify({ alg: b.kdf.alg, iter: b.kdf.iter, salt: b.kdf.salt, check: b.kdf.check }), now, email),
			env.DB.prepare(`UPDATE creds SET key_enc = (SELECT json_extract(value, '$.key_enc') FROM json_each(?) WHERE json_extract(value, '$.id') = creds.id),
				version = version + 1, updated = ?, updated_by = ?`).bind(JSON.stringify(keys.map(k => ({ id: String(k.id), key_enc: k.key_enc }))), now, email),
			hist(env, VAULT_ROW, 'passphrase changed', email, `${keys.length} key(s) re-sealed`),
		]);
		return json({ kdf: await getKdf(env) });
	}

	if (path === 'creds' && method === 'GET') {
		const { results } = await env.DB.prepare(`SELECT ${PUBLIC_COLS} FROM creds ORDER BY added, CAST(id AS INTEGER)`).all();
		return json({ email, kdfVersion: kdf.version, records: results });
	}

	if (path === 'export' && method === 'GET') {                   // every sealed key: backup or passphrase change
		const { results } = await env.DB.prepare(`SELECT ${PUBLIC_COLS}, key_enc FROM creds ORDER BY added, CAST(id AS INTEGER)`).all();
		if (results.length) await histMany(env, results.map(r => r.id), 'key exported', email, String(url.searchParams.get('why') || '').slice(0, 120)).run();
		return json({ email, kdf, records: results });
	}

	if (path === 'history' && method === 'GET') {                  // the vault's own events
		const { results } = await env.DB.prepare('SELECT t, ev, by, note FROM history WHERE cred_id = ? ORDER BY seq').bind(VAULT_ROW).all();
		return json({ history: results });
	}

	if (path === 'import' && method === 'POST') {
		const b = await body(request);
		const source = String(b.source || '').slice(0, 120);
		const list = Array.isArray(b.records) ? b.records : [];
		if (list.length > IMPORT_MAX) throw new HttpError(413, `At most ${IMPORT_MAX} sets per request`);
		if (Number(b.kdfVersion) !== kdf.version) throw new HttpError(409, 'The team passphrase was changed: unlock again', { code: 'rekeyed' });
		const { results: existing } = await env.DB.prepare('SELECT id, addr, key_crc FROM creds').all();
		const byId = new Map(existing.map(r => [r.id, r])), byAddr = new Map(existing.map(r => [r.addr, r]));
		const res = { added: [], identical: [], conflicts: [], invalid: [] }, rows = [], now = new Date().toISOString();
		for (let i = 0; i < list.length; i++) {
			const { rec, errors } = normalize(list[i] || {});
			if (Object.keys(errors).length) { res.invalid.push({ index: i, id: rec.id, errors }); continue; }
			const cur = byId.get(rec.id), other = byAddr.get(rec.addr);
			if (cur) {
				if (cur.addr === rec.addr && cur.key_crc === rec.key_crc) res.identical.push(rec.id);
				else res.conflicts.push({ id: rec.id, why: cur.addr !== rec.addr ? 'ADDR differs from the vault' : 'KEY differs from the vault' });
				continue;
			}
			if (other) { res.conflicts.push({ id: rec.id, why: `ADDR ${rec.addr} already belongs to ID ${other.id}` }); continue; }
			byId.set(rec.id, rec); byAddr.set(rec.addr, rec);
			rows.push(rec); res.added.push(rec.id);
		}
		if (rows.length) {
			await env.DB.batch([
				env.DB.prepare(`INSERT INTO creds (id, addr, key_enc, key_crc, rconf, status, sn, fw, assigned, note, source, added, updated, updated_by)
					SELECT json_extract(value, '$.id'), json_extract(value, '$.addr'), json_extract(value, '$.key_enc'), json_extract(value, '$.key_crc'),
						json_extract(value, '$.rconf'), json_extract(value, '$.status'), json_extract(value, '$.sn'), json_extract(value, '$.fw'),
						json_extract(value, '$.assigned'), json_extract(value, '$.note'), ?, ?, ?, ? FROM json_each(?)`)
					.bind(source, now, now, email, JSON.stringify(rows)),
				histMany(env, res.added, 'imported', email, source),
			]);
		}
		return json(res);
	}

	const m = path.match(/^creds\/(\d{1,10})(?:\/(key|track|history))?$/);
	if (!m) throw new HttpError(404, 'Not found');
	const id = m[1], sub = m[2] || '';
	const cur = await getRec(env, id);
	if (!cur) throw new HttpError(404, `ID ${id} is not in the team vault`, { code: 'unknown_id' });

	if (sub === 'history' && method === 'GET') {
		const { results } = await env.DB.prepare('SELECT t, ev, by, note FROM history WHERE cred_id = ? ORDER BY seq').bind(id).all();
		return json({ history: results });
	}
	if (sub === 'key' && method === 'GET') {
		const row = await env.DB.prepare('SELECT key_enc FROM creds WHERE id = ?').bind(id).first();
		await hist(env, id, 'key read', email, String(url.searchParams.get('why') || '').slice(0, 120)).run();
		return json({ id, key_enc: row.key_enc, kdfVersion: kdf.version });
	}
	if (sub === 'track' && method === 'POST') {
		const b = await body(request);
		const sn = String(b.sn || '').slice(0, 64), fw = String(b.fw || '').slice(0, 96), now = new Date().toISOString();
		const late = b.t && Date.parse(b.t) < Date.now() - 300e3 ? ` (recorded ${String(b.t).slice(0, 16).replace('T', ' ')} UTC)` : '';
		if (b.kind === 'provisioned') {
			const prev = cur.status === 'in use' && cur.sn && cur.sn !== sn ? ` (was on ${cur.sn})` : '';
			await env.DB.batch([
				env.DB.prepare("UPDATE creds SET status = 'in use', sn = ?, fw = ?, assigned = ?, updated = ?, updated_by = ?, version = version + 1 WHERE id = ?").bind(sn, fw, now.slice(0, 10), now, email, id),
				hist(env, id, 'provisioned', email, `${sn || 'unit'}${prev}${late}`),
			]);
		} else if (b.kind === 'erased') {
			const was = cur.sn || sn;
			await env.DB.batch([
				env.DB.prepare("UPDATE creds SET status = 'free', sn = '', fw = '', assigned = '', updated = ?, updated_by = ?, version = version + 1 WHERE id = ?").bind(now, email, id),
				hist(env, id, 'erased', email, `${was ? `from ${was}` : ''}${late}`),
			]);
		} else throw new HttpError(400, 'kind must be provisioned or erased');
		return json({ record: await getRec(env, id) });
	}
	if (!sub && method === 'PATCH') {
		const b = await body(request);
		if (Number(b.version) !== cur.version) throw changedBy(id, cur);
		const fields = EDITABLE.filter(f => f in b);
		const newKey = fields.includes('key_enc');
		if (newKey && Number(b.kdfVersion) !== kdf.version) throw new HttpError(409, 'The team passphrase was changed: unlock again', { code: 'rekeyed' });
		const row = await env.DB.prepare('SELECT key_enc FROM creds WHERE id = ?').bind(id).first();
		const { rec, errors } = normalize({ ...cur, key_enc: row.key_enc, ...Object.fromEntries(fields.map(f => [f, b[f]])), key_crc: newKey ? b.key_crc : cur.key_crc });
		const bad = [...fields, ...(newKey ? ['key_crc'] : [])].filter(f => errors[f]);
		if (bad.length) throw new HttpError(400, bad.map(f => `${f.toUpperCase()}: ${errors[f]}`).join('; '));
		if (rec.addr !== cur.addr) {
			const other = await env.DB.prepare('SELECT id FROM creds WHERE addr = ? AND id <> ?').bind(rec.addr, id).first();
			if (other) throw new HttpError(409, `ADDR ${rec.addr} already belongs to ID ${other.id}`);
		}
		const changed = fields.filter(f => f === 'key_enc' ? rec.key_enc !== row.key_enc : rec[f] !== cur[f]);
		if (!changed.length) return json({ record: cur });
		const now = new Date().toISOString();
		const [upd] = await env.DB.batch([
			env.DB.prepare('UPDATE creds SET addr = ?, key_enc = ?, key_crc = ?, rconf = ?, status = ?, sn = ?, fw = ?, assigned = ?, note = ?, updated = ?, updated_by = ?, version = version + 1 WHERE id = ? AND version = ?')
				.bind(rec.addr, rec.key_enc, rec.key_crc, rec.rconf, rec.status, rec.sn, rec.fw, rec.assigned, rec.note, now, email, id, cur.version),
			// logged only when the UPDATE above matched (same version)
			env.DB.prepare('INSERT INTO history (cred_id, t, ev, by, note) SELECT ?, ?, ?, ?, ? WHERE changes() > 0').bind(id, now,
				changed.includes('status') ? `status: ${cur.status} → ${rec.status}` : 'edited', email,
				changed.filter(f => f !== 'status').map(f => f === 'key_enc' ? `key (CRC-32 ${rec.key_crc})` : f).join(', ')),
		]);
		if (upd && upd.meta && upd.meta.changes === 0) throw changedBy(id, await getRec(env, id));
		return json({ record: await getRec(env, id) });
	}
	if (!sub && method === 'DELETE') {
		if (Number(url.searchParams.get('version')) !== cur.version) throw changedBy(id, cur);
		await env.DB.batch([env.DB.prepare('DELETE FROM creds WHERE id = ?').bind(id), hist(env, id, 'deleted', email, `${cur.addr} ${cur.status}${cur.sn ? ' on ' + cur.sn : ''}`)]);
		return json({ ok: true });
	}
	throw new HttpError(405, 'Method not allowed');
}
