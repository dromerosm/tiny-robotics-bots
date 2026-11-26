# home-energy: the bridge's energy history on Cloudflare

The bridge keeps only the last 48 hours and 62 days. This Worker keeps all of it in D1, with a price table, and shows it at `energy.diegoromero.es`.

- Every hour the bridge sends its `/api/energy` JSON to `POST /ingest`, with `Authorization: Bearer <INGEST_TOKEN>`.
- Each hour and day is an upsert that keeps the larger value. If a send is lost, the next one covers it. A bridge that lost its history cannot lower what is stored.
- The cost is worked out when it is read: Wh / 1000 × the price that holds on that day. Only the energy term, with no taxes and no power term. If you add or fix a price, the whole history changes with it.
- The page and `/api/*` are for you only, behind Cloudflare Access. `/ingest` is outside Access and checks the token.

## API

| Path | What it gives |
|---|---|
| `GET /api/days?from=2026-11-01&to=2026-11-30` | Wh and EUR per device and day |
| `GET /api/hours?from=2026-11-25&to=2026-11-26` | Wh and EUR per device and hour (local time, Europe/Madrid) |
| `GET /api/prices` | The price table |

## Prices

A price holds from its day until the next row. To add a new one:

```bash
npx wrangler d1 execute home-energy --remote --command "INSERT INTO prices VALUES ('2027-01-01', 0.15, 'new contract')"
```

## Run it locally

```bash
printf 'INGEST_TOKEN=%s\n' "$(openssl rand -hex 16)" > .dev.vars
npx wrangler@4.135.0 d1 migrations apply home-energy --local
npx wrangler@4.135.0 dev
```

## Deploy

```bash
npx wrangler@4.135.0 d1 create home-energy        # put its id in wrangler.jsonc
npx wrangler@4.135.0 d1 migrations apply home-energy --remote
npx wrangler@4.135.0 secret put INGEST_TOKEN
npx wrangler@4.135.0 deploy
```

Then add a Cloudflare Access application for `energy.diegoromero.es`, with a bypass for `/ingest`.

The zone has a WAF custom rule that challenges every HTTP/1.1 request. The bridge's HTTP client only speaks HTTP/1.1 and cannot pass a challenge, so it gets a 403. The first custom rule in the zone skips the rest for `POST energy.diegoromero.es/ingest`. The Worker's token still protects that path.
