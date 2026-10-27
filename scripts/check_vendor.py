"""Verify that vendor/canbus_analyzer_aeb/*.py are byte-identical to can-bus-analyzer commit 27da004.

Checks the SHA-256 and the git blob id of each file against the values recorded in
vendor/canbus_analyzer_aeb/PROVENANCE.md (which anyone can re-derive with
`git rev-parse 27da004:canbus_analyzer/<file>` in a clone of the public repository).

Run:  python -I scripts/check_vendor.py
"""
import hashlib
import sys
from pathlib import Path

VENDOR = Path(__file__).resolve().parents[1] / "vendor" / "canbus_analyzer_aeb"
EXPECTED = {   # file: (git blob id at 27da004, sha256)
    "aeb_function.py": ("4474f26f8f082630c35fb6452b38b053207aa151",
                        "c90dff394375959636cf4ddac63bc53e00e0d8e9466567f73d3c060d5d188c18"),
    "analyzer.py": ("5583db1396d5b115d6801527ef26ee191ac6ee72",
                    "71cee951515fca74d14be224043c5c27d91906ce2e2d1ae26f885d367bac3518"),
    "config.py": ("3c4c9a759f0651665dceccb76c96bad7dca39c6c",
                  "31d760784d474d7f4abfbe3e8311046ea5c31629d11654c18931c4c11e12703d"),
}


def git_blob_id(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def main() -> int:
    ok = True
    for name, (blob, sha) in EXPECTED.items():
        data = (VENDOR / name).read_bytes()
        got_blob, got_sha = git_blob_id(data), hashlib.sha256(data).hexdigest()
        good = got_blob == blob and got_sha == sha
        ok &= good
        print(f"{'OK  ' if good else 'FAIL'} {name}: git blob {got_blob}  sha256 {got_sha}")
    print("vendored files identical to upstream commit 27da004" if ok else "MISMATCH - vendored code was modified")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
