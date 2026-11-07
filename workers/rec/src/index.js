// The robot's recordings (14-record), in R2.
//
// The robot (header X-Robot-Token, the secret REC_TOKEN) can only add:
//   POST /rec                 start a recording: {"id": "20261107-114029-3fa2"}
//   PUT  /rec/<id>/<n>        piece n (0, 1, 2...): 16-bit PCM, 16 kHz, mono
//   POST /rec/<id>/end        the recording is over
// The Mac app (header Authorization: Bearer <READ_TOKEN>) reads and deletes:
//   GET    /rec               [{id, seconds, pieces, missing, ended}], newest first
//   GET    /rec/<id>.wav      the whole recording as one WAV
//   DELETE /rec/<id>
//
// The Worker names each recording from its own clock, so the robot needs no
// time. A piece that never came is silence in the WAV, so the times after
// it stay right.
const RATE = 16000;
const MAX_PIECE = 1 << 20; // 1 MB, about 32 s
const ID = /^\d{8}-\d{6}-[0-9a-f]{4}$/;

const json = (data, status = 200) =>
  new Response(JSON.stringify(data), { status, headers: { "content-type": "application/json" } });
const text = (msg, status) => new Response(msg + "\n", { status });

async function same(a, b) {
  if (!a || !b) return false;
  const enc = new TextEncoder();
  const [x, y] = [enc.encode(a), enc.encode(b)];
  return x.byteLength === y.byteLength && crypto.subtle.timingSafeEqual(x, y);
}

const robot = (req, env) => same(req.headers.get("x-robot-token"), env.REC_TOKEN);
const reader = (req, env) =>
  same((req.headers.get("authorization") || "").replace(/^Bearer /, ""), env.READ_TOKEN);

function newId() {
  const t = new Date().toISOString(); // 2026-11-07T11:40:29.123Z
  const rand = [...crypto.getRandomValues(new Uint8Array(2))]
    .map((b) => b.toString(16).padStart(2, "0"))
    .join("");
  return `${t.slice(0, 10).replaceAll("-", "")}-${t.slice(11, 19).replaceAll(":", "")}-${rand}`;
}

async function listAll(bucket, prefix) {
  const out = [];
  let cursor;
  do {
    const page = await bucket.list({ prefix, cursor });
    out.push(...page.objects);
    cursor = page.truncated ? page.cursor : undefined;
  } while (cursor);
  return out;
}

// The pieces of one recording, in order, with the gaps.
async function pieces(bucket, id) {
  const objs = await listAll(bucket, `rec/${id}/`);
  const parts = new Map();
  let ended = false;
  for (const o of objs) {
    const name = o.key.slice(`rec/${id}/`.length);
    if (name === "end") ended = true;
    const m = name.match(/^(\d{5})\.pcm$/);
    if (m) parts.set(Number(m[1]), o);
  }
  const last = parts.size ? Math.max(...parts.keys()) : -1;
  const size = parts.get(0)?.size || parts.values().next().value?.size || 0;
  const list = [];
  for (let n = 0; n <= last; n++) list.push(parts.get(n) || { missing: true, size });
  return { list, ended };
}

function wavHeader(bytes) {
  const h = new DataView(new ArrayBuffer(44));
  const s = (o, str) => [...str].forEach((c, i) => h.setUint8(o + i, c.charCodeAt(0)));
  s(0, "RIFF");
  h.setUint32(4, 36 + bytes, true);
  s(8, "WAVEfmt ");
  h.setUint32(16, 16, true);
  h.setUint16(20, 1, true); // PCM
  h.setUint16(22, 1, true); // mono
  h.setUint32(24, RATE, true);
  h.setUint32(28, RATE * 2, true);
  h.setUint16(32, 2, true);
  h.setUint16(34, 16, true);
  s(36, "data");
  h.setUint32(40, bytes, true);
  return new Uint8Array(h.buffer);
}

async function wav(env, id, ctx) {
  const { list } = await pieces(env.REC, id);
  if (!list.length) return text("not found", 404);
  const bytes = list.reduce((a, p) => a + p.size, 0);
  const { readable, writable } = new FixedLengthStream(44 + bytes);
  const w = writable.getWriter();
  ctx.waitUntil(
    (async () => {
      await w.write(wavHeader(bytes));
      for (const p of list) {
        if (p.missing) {
          await w.write(new Uint8Array(p.size));
          continue;
        }
        const obj = await env.REC.get(p.key);
        const r = obj.body.getReader();
        for (let c = await r.read(); !c.done; c = await r.read()) await w.write(c.value);
      }
      await w.close();
    })().catch((e) => w.abort(e)),
  );
  return new Response(readable, {
    headers: {
      "content-type": "audio/wav",
      "content-disposition": `attachment; filename="stackchan-${id}.wav"`,
    },
  });
}

async function index(env) {
  const objs = await listAll(env.REC, "rec/");
  const ids = [...new Set(objs.map((o) => o.key.split("/")[1]))].filter((i) => ID.test(i));
  const out = [];
  for (const id of ids) {
    const { list, ended } = await pieces(env.REC, id);
    out.push({
      id,
      seconds: Math.round(list.reduce((a, p) => a + p.size, 0) / (RATE * 2)),
      pieces: list.length,
      missing: list.filter((p) => p.missing).length,
      ended,
    });
  }
  return json(out.sort((a, b) => b.id.localeCompare(a.id)));
}

export default {
  async fetch(req, env, ctx) {
    const url = new URL(req.url);
    const p = url.pathname.split("/").filter(Boolean); // ["rec", id, n]
    if (p[0] !== "rec" || p.length > 3) return text("not found", 404);
    const id = p[1]?.replace(/\.wav$/, "");
    if (id && !ID.test(id)) return text("bad id", 400);

    // The robot
    if (req.method === "POST" || req.method === "PUT") {
      if (!(await robot(req, env))) return text("forbidden", 403);
      if (req.method === "POST" && p.length === 1) return json({ id: newId() }, 201);
      if (req.method === "POST" && p[2] === "end") {
        await env.REC.put(`rec/${id}/end`, new Date().toISOString());
        return text("ok", 200);
      }
      if (req.method === "PUT" && /^\d{1,5}$/.test(p[2] || "")) {
        const len = Number(req.headers.get("content-length"));
        if (!len || len > MAX_PIECE || len % 2) return text("bad length", 400);
        const key = `rec/${id}/${p[2].padStart(5, "0")}.pcm`;
        await env.REC.put(key, req.body);
        return text("ok", 200);
      }
      return text("not found", 404);
    }

    // The Mac app
    if (!(await reader(req, env))) return text("forbidden", 403);
    if (req.method === "GET" && p.length === 1) return index(env);
    if (req.method === "GET" && p.length === 2 && p[1].endsWith(".wav")) return wav(env, id, ctx);
    if (req.method === "DELETE" && p.length === 2) {
      const objs = await listAll(env.REC, `rec/${id}/`);
      if (objs.length) await env.REC.delete(objs.map((o) => o.key));
      return text(`deleted ${objs.length}`, 200);
    }
    return text("not found", 404);
  },
};
