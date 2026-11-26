-- The energy the bridge measured, in Wh, by device and by local hour and day.
CREATE TABLE devices (
  id   TEXT PRIMARY KEY, -- the plug's Tuya id
  name TEXT NOT NULL
);
CREATE TABLE hours (
  dev TEXT NOT NULL REFERENCES devices(id),
  t   TEXT NOT NULL, -- local hour, "2026-11-26T17"
  wh  REAL NOT NULL,
  PRIMARY KEY (dev, t)
);
CREATE TABLE days (
  dev TEXT NOT NULL REFERENCES devices(id),
  d   TEXT NOT NULL, -- local day, "2026-11-26"
  wh  REAL NOT NULL,
  PRIMARY KEY (dev, d)
);
-- The price of the energy term (no taxes, no power term). A price holds from
-- its day until the next row. Costs are worked out when they are read, so a new
-- or corrected price changes the whole history.
CREATE TABLE prices (
  valid_from TEXT PRIMARY KEY, -- local day, "2026-01-01"
  eur_kwh    REAL NOT NULL CHECK (eur_kwh >= 0),
  note       TEXT
);
INSERT INTO prices VALUES ('2000-01-01', 0.163463, 'CONFIG_HOME_PRICE_UEUR_KWH in the firmware');
