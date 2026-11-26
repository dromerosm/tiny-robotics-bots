"""Copy some namespaces of an NVS dump into a CSV for nvs_partition_gen.py.

Runs inside the ESP-IDF container:
  python nvs_move.py <nvs-dump.bin> <out-dir> <namespace> [<namespace> ...]
Writes <out-dir>/nvs.csv and one .bin file per blob. Prints what it keeps.
Values are never printed, only sizes: the dump holds HomeKit's private keys.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.environ["IDF_PATH"], "components/nvs_flash/nvs_partition_tool"))
from nvs_parser import NVS_Partition  # noqa: E402

INT_TYPES = {
    "uint8_t": "u8", "int8_t": "i8", "uint16_t": "u16", "int16_t": "i16",
    "uint32_t": "u32", "int32_t": "i32", "uint64_t": "u64", "int64_t": "i64",
}


def payload(entry):
    data = b"".join(bytes(c.raw) for c in entry.children)
    return data[: entry.data["size"]]


def main():
    dump, out, keep = sys.argv[1], sys.argv[2], set(sys.argv[3:])
    nvs = NVS_Partition("nvs", bytearray(open(dump, "rb").read()))
    written = [e for p in nvs.pages for e in p.entries if e.state == "Written"]
    names = {e.data["value"]: e.key for e in written if e.metadata["namespace"] == 0}
    by_ns = {}
    for e in written:
        ns = names.get(e.metadata["namespace"])
        if e.metadata["namespace"] != 0 and ns in keep:
            by_ns.setdefault(ns, []).append(e)

    os.makedirs(out, exist_ok=True)
    rows = ["key,type,encoding,value"]
    for ns in sorted(by_ns):
        rows.append(f"{ns},namespace,,")
        entries = by_ns[ns]
        chunks = {}
        for e in entries:
            if e.metadata["type"] == "blob_data":
                chunks.setdefault(e.key, {})[e.metadata["chunk_index"]] = payload(e)
        for e in entries:
            kind = e.metadata["type"]
            if kind in INT_TYPES:
                rows.append(f"{e.key},data,{INT_TYPES[kind]},{e.data['value']}")
                print(f"  {ns}:{e.key} {kind}")
            elif kind == "string":
                text = payload(e).rstrip(b"\0").decode()
                if "," in text or '"' in text or "\n" in text:
                    sys.exit(f"cannot put {ns}:{e.key} in a CSV")
                rows.append(f"{e.key},data,string,{text}")
                print(f"  {ns}:{e.key} string, {len(text)} chars")
            elif kind in ("blob_index", "blob"):
                if kind == "blob":
                    value = payload(e)
                else:
                    start, count = e.data["chunk_start"], e.data["chunk_count"]
                    parts = chunks.get(e.key, {})
                    value = b"".join(parts[start + i] for i in range(count))
                    if len(value) != e.data["size"]:
                        sys.exit(f"{ns}:{e.key}: {len(value)} bytes, expected {e.data['size']}")
                path = os.path.join(out, f"{ns}.{e.key}.bin")
                open(path, "wb").write(value)
                rows.append(f"{e.key},file,binary,{os.path.abspath(path)}")
                print(f"  {ns}:{e.key} blob, {len(value)} bytes")
    missing = keep - set(by_ns)
    if missing:
        print(f"not in the dump: {', '.join(sorted(missing))}")
    open(os.path.join(out, "nvs.csv"), "w").write("\n".join(rows) + "\n")


main()
