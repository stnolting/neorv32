import re
import xmltodict

## todo -- generate header banners and comments
## todo -- handle "holes" in peripheral register offsets

with open('../svd/neorv32.svd') as fd:
    dict = xmltodict.parse(fd.read(), force_list={'field', 'register'})

periList = dict['device']['peripherals']['peripheral']
out = open('../lib/include/neorv32_svd.h', 'w')

out.write("#ifndef neorv32_svd_h\n")
out.write("#define neorv32_svd_h\n")
out.write("\n")
out.write("#ifdef NEORV32_SVD_HEADER\n")
out.write("#pragma message \"*** using 'neorv32_svd_h'\"\n")
out.write("#endif\n")
out.write("\n")

for peri in periList:

    pname = peri['name']
    out.write("/* ---- %s ---- */\n" % (pname))
    out.write("\n")
    ptype = re.sub('\d+$', '', pname)

    if '@derivedFrom' not in peri:

        regList = peri['registers']['register']
        out.write("typedef volatile struct __attribute__((packed,aligned(4))) {\n")

        for reg in regList:
            cs = "const " if 'access' in reg and reg['access'] == 'read-only' else ""
            dim = "" if 'dim' not in reg else f"[{reg['dim']}]"
            out.write("    %suint32_t %s%s;\n" % (cs, reg['name'], dim))

        out.write("} neorv32_%s_t;\n" % (ptype.lower()))
        out.write("\n")

    out.write("#define NEORV32_%s_BASE %s\n" % (pname, peri['baseAddress']))
    out.write("#define NEORV32_%s ((neorv32_%s_t*) (NEORV32_%s_BASE))\n" % (pname, ptype.lower(), pname))
    out.write("\n")

    if '@derivedFrom' not in peri:

        for reg in regList:

            if 'fields' not in reg: continue

            rname = reg['name']
            out.write("enum NEORV32_%s_%s_enum {\n" % (ptype, rname))
            fldList = reg['fields']['field']

            for fld in fldList:
                fname = fld['name']
                br = list(map(lambda s: int(s), re.findall('\d+', fld['bitRange'])))
                if br[0] == br[1]:
                    out.write("    %s = %d,\n" % (fname, br[0]))
                else:
                    for i in range(br[0] - br[1] + 1):
                        out.write("    %s%d = %d,\n" % (fname, i, br[1] + i))

            out.write("};\n")
            out.write("\n")

out.write("#endif // neorv32_svd_h\n")

out.close()