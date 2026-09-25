"""Tiny KiCad 7 symbol-library reader (S-expressions) - enough to pull a symbol
out of the stock libraries, list its pins and embed it in a schematic."""
import re, os

SYMDIR = os.environ.get("KICAD_SYMBOL_DIR", "/usr/share/kicad/symbols")


def tokenize(s):
    return re.findall(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+', s)


def parse(s):
    toks = tokenize(s)
    pos = 0
    def rd():
        nonlocal pos
        t = toks[pos]; pos += 1
        if t == "(":
            lst = []
            while toks[pos] != ")":
                lst.append(rd())
            pos += 1
            return lst
        return t
    return rd()


def dump(x, ind=0):
    """Serialise a parsed S-expression back to KiCad text."""
    if not isinstance(x, list):
        return x
    head = []
    rest = []
    for e in x:
        if isinstance(e, list) or rest:
            rest.append(e)
        else:
            head.append(e)
    s = "(" + " ".join(head)
    for e in rest:
        if isinstance(e, list):
            s += "\n" + "  " * (ind + 1) + dump(e, ind + 1)
        else:
            s += " " + e
    return s + ")"


_cache = {}


def load_lib(lib):
    if lib not in _cache:
        with open(os.path.join(SYMDIR, lib + ".kicad_sym")) as fh:
            _cache[lib] = parse(fh.read())
    return _cache[lib]


def find_symbol(lib, name):
    root = load_lib(lib)
    for e in root:
        if isinstance(e, list) and e[0] == "symbol" and e[1].strip('"') == name:
            return e
    raise KeyError(f"{lib}:{name}")


def resolve(lib, name):
    """Return the symbol with 'extends' flattened (derived symbols copy the parent's units)."""
    sym = find_symbol(lib, name)
    ext = [e for e in sym if isinstance(e, list) and e[0] == "extends"]
    if not ext:
        return sym
    parent = resolve(lib, ext[0][1].strip('"'))
    props = [e for e in sym if isinstance(e, list) and e[0] == "property"]
    pname = parent[1].strip('"')
    out = ["symbol", f'"{name}"']
    out += [e for e in parent[2:] if isinstance(e, list) and e[0] not in ("property", "symbol")]
    out += props
    for e in parent[2:]:
        if isinstance(e, list) and e[0] == "symbol":
            sub = list(e)
            sub[1] = '"' + sub[1].strip('"').replace(pname, name, 1) + '"'
            out.append(sub)
    return out


def pins(sym):
    """[(number, name, electrical_type, x, y, angle, unit)]"""
    res = []
    for e in sym:
        if isinstance(e, list) and e[0] == "symbol":
            m = re.match(r'.*_(\d+)_(\d+)$', e[1].strip('"'))
            unit = int(m.group(1)) if m else 0
            for p in e:
                if isinstance(p, list) and p[0] == "pin":
                    at = [q for q in p if isinstance(q, list) and q[0] == "at"][0]
                    nm = [q for q in p if isinstance(q, list) and q[0] == "name"][0][1].strip('"')
                    num = [q for q in p if isinstance(q, list) and q[0] == "number"][0][1].strip('"')
                    res.append((num, nm, p[1], float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0, unit))
    return res


def prop(sym, key):
    for e in sym:
        if isinstance(e, list) and e[0] == "property" and e[1].strip('"') == key:
            return e[2].strip('"')
    return None
