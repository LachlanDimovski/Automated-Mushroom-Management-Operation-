"""Split each multi-group OBJ into one OBJ per group (Fortress ignores OBJ submesh names) and point model.sdf at them."""
import glob, os, re, sys
for obj in glob.glob(os.path.join(sys.argv[1], '*/*/mesh/*.obj')):
    lines = open(obj).read().splitlines()
    groups = [l[2:].strip() for l in lines if l.startswith('o ')]
    if len(groups) < 2:
        continue
    V = {'v': [], 'vt': [], 'vn': []}
    faces, cur = {}, None
    for l in lines:
        t = l.split(' ', 1)
        if t[0] in V: V[t[0]].append(t[1])
        elif t[0] == 'o': cur = t[1].strip(); faces[cur] = []
        elif t[0] == 'f': faces[cur].append(t[1].split())
    mdir = os.path.dirname(obj); base = os.path.splitext(os.path.basename(obj))[0]
    sdf_p = os.path.join(os.path.dirname(mdir), 'model.sdf'); sdf = open(sdf_p).read()
    for g, fs in faces.items():
        maps = {k: {} for k in V}; out = {k: [] for k in V}; fl = []
        for f in fs:
            nf = []
            for c in f:
                idx = c.split('/'); r = []
                for k, i in zip(('v', 'vt', 'vn'), idx):
                    if not i: r.append(''); continue
                    i = int(i)
                    if i not in maps[k]: maps[k][i] = len(out[k]) + 1; out[k].append(V[k][i - 1])
                    r.append(str(maps[k][i]))
                nf.append('/'.join(r))
            fl.append('f ' + ' '.join(nf))
        fn = re.sub(r'[^A-Za-z0-9_]', '_', g) + '.obj'
        with open(os.path.join(mdir, fn), 'w') as fh:
            for k in ('v', 'vt', 'vn'): fh.writelines(f'{k} {x}\n' for x in out[k])
            fh.write('\n'.join(fl) + '\n')
        sdf = sdf.replace(f'mesh/{base}.obj</uri><submesh><name>{g}</name></submesh>', f'mesh/{fn}</uri>')
    open(sdf_p, 'w').write(sdf); os.remove(obj); print('split', base, len(faces))
