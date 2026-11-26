// Checks the JWT that Cloudflare Access adds to every request it lets through
// (header Cf-Access-Jwt-Assertion). Access already stops everyone else at the
// edge; this check means that a mistake in the Access setup does not open the
// data too.
//
// The token is RS256, signed with one of the team's keys, published at
// <team>/cdn-cgi/access/certs. It must name our application (aud), come from
// our team (iss), and still be valid (exp, nbf).

const keys = new Map(); // kid -> CryptoKey
let fetchedAt = 0;

function b64url(text) {
  const s = text.replace(/-/g, "+").replace(/_/g, "/");
  const bin = atob(s + "=".repeat((4 - (s.length % 4)) % 4));
  return Uint8Array.from(bin, (c) => c.charCodeAt(0));
}

async function key(team, kid, fetcher) {
  // Fetch the keys again when the kid is new (Access rotates them), at most once a minute.
  if (!keys.has(kid) && Date.now() - fetchedAt > 60_000) {
    fetchedAt = Date.now();
    const r = await fetcher(`${team}/cdn-cgi/access/certs`);
    if (r.ok) {
      const { keys: list = [] } = await r.json();
      for (const jwk of list) {
        if (jwk.kty === "RSA" && jwk.kid) {
          keys.set(jwk.kid, await crypto.subtle.importKey(
            "jwk", jwk, { name: "RSASSA-PKCS1-v1_5", hash: "SHA-256" }, false, ["verify"]));
        }
      }
    }
  }
  return keys.get(kid);
}

// True when the request carries a valid Access token for this application.
// team: "https://<team>.cloudflareaccess.com"; aud: the application's AUD tag.
export async function accessOk(req, team, aud, fetcher = fetch, now = Date.now() / 1000) {
  const jwt = req.headers.get("Cf-Access-Jwt-Assertion") || "";
  const parts = jwt.split(".");
  if (!team || !aud || parts.length !== 3) {
    return false;
  }
  try {
    const head = JSON.parse(new TextDecoder().decode(b64url(parts[0])));
    const claims = JSON.parse(new TextDecoder().decode(b64url(parts[1])));
    if (head.alg !== "RS256" || typeof head.kid !== "string") {
      return false;
    }
    const k = await key(team, head.kid, fetcher);
    if (!k) {
      return false;
    }
    const signed = new TextEncoder().encode(`${parts[0]}.${parts[1]}`);
    if (!(await crypto.subtle.verify("RSASSA-PKCS1-v1_5", k, b64url(parts[2]), signed))) {
      return false;
    }
    const auds = Array.isArray(claims.aud) ? claims.aud : [claims.aud];
    return auds.includes(aud) && claims.iss === team &&
      typeof claims.exp === "number" && claims.exp > now &&
      (claims.nbf === undefined || claims.nbf <= now + 60);
  } catch {
    return false;
  }
}

// For the tests only.
export function _reset() {
  keys.clear();
  fetchedAt = 0;
}
