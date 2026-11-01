# -*- coding: utf-8 -*-
"""Педальный генератор — стенд v5. Параметрический генератор для Blender 4.5+/5.x.

Запуск внутри Blender: Scripting -> Open -> Run Script (или runpy.run_path(...)).
Все размеры в мм. Оси сборки: X — вдоль станка (каретка X=0, втулка +X),
Y — поперёк (цепь справа, +Y), Z — вверх, пол Z=0.
Выход: stl_individual/*.stl (одна связная деталь на файл, Z вверх = ориентация печати),
bike_generator_stand_assembled.stl в корне версии (сборка, НЕ для печати),
print_manifest.json/csv, assembly_report.json, checks.json, torque_plate_steel_30x6_x2.dxf, preview_*.png, .blend.
v5: свободностоящий стенд без крепления к полу; широкое основание 400 мм под кареткой;
меньше деталей (объединены поперечина+мост, 1 проставка на сторону, нет барашков и PLA-шаблонов).
"""
import bpy, bmesh, math, os, json, struct, csv
from mathutils import Matrix, Vector

PROJECT = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(PROJECT, "stl_individual")
DO_RENDER = globals().get("DO_RENDER", True)

# ------------------------- ПАРАМЕТРЫ -------------------------
# Каретка / башня
BB_Z        = 250.0   # ось каретки над полом (v4: 205). Поднята ради зазора педалей/обуви над крыльями
SLEEVE_ID   = 41.1    # внутр. Ø гильзы BB86 (подобрать по тест-кольцам)
SLEEVE_OD   = 60.0
SLEEVE_L    = 86.5    # ширина корпуса BB86 — строго
CLAMP_ID    = 60.4
CLAMP_OD    = 84.0
TOWER_T     = 44.0    # толщина башни по Y (±22)
BAR_Z0, BAR_Z1 = 90.0, 108.0        # полка башни лежит на опорах основания (Z=90)
TOWER_BOLT_X = 64.0                 # 4 x M8x80 DIN 912 поперёк: головка в зенковке одной щеки, гайка в шестигранном кармане другой
TOWER_BOLT_Z = (100.0, 126.0)
# Основание кареточного модуля: две ОДИНАКОВЫЕ половины (левая = правая, повёрнутая на 180° вокруг Z)
TUBE_D = 20.0; TUBE_WALL = 2.0; TUBE_HOLE = 20.4; TUBE_Y = 24.0; TUBE_Z = 22.0
TUBE_X0 = -100.0; TUBE_LEN = 400.0  # трубы: X -100..300
BASE_W   = 200.0      # половина Y 0..200 -> колея 400 мм
SPINE_Y  = 46.0       # продольный брус вокруг трубы: Y 0..46 (|Y|<48 — зона шатунов снаружи)
SPINE_H  = 44.0
PIER_X   = (20.0, 100.0); PIER_H = BAR_Z0        # опоры под полку башни (X ±20..100)
WALL_Y   = (TOWER_T/2 + 0.3, 42.0); WALL_TOP = 140.0   # щёки, зажимающие башню (зазор 0.3/сторона)
ARM_X    = (60.0, 100.0); ARM_H0 = 30.0; ARM_H1 = 16.0; ARM_Y0 = 40.0   # крылья-выносы (сужаются к концу)
GRUB_X   = 0.0; GRUB_NUT_Y0 = 37.2    # установочный M6x16 сбоку между опорами, гайка в кармане, вход кармана сверху
STRAP_X  = (65.0, 95.0); STRAP_Y = (172.0, 178.0)   # прорезь под стропу/верёвку к стулу
# Втулка (замеры владельца + условные величины)
HUB_X = 325.0; HUB_Z = 160.0           # (v4: 115)
OLD = 145.0                            # приблизительно; уточнить по опорным плоскостям
AXLE_D = 11.7; AXLE_FLAT = 9.7
AXLE_LEN = 210.0                       # условно; нужна реальная длина свободной резьбы
HUB_D = 150.0; HUB_ACTUAL_D = 139.0; HUB_W = 80.0; HUB_CAP_D = 120.0; HUB_BODY_W = 95.0; HUB_SHOULDER_D = 30.0
DO_T = 16.0; DO_H = HUB_Z + 42.0       # толщина/высота боковины
TQ_W = 31.0; TQ_DEPTH = 6.0; TQ_Z0 = HUB_Z-102.0; TQ_BOLT_Z = (HUB_Z-62.0, HUB_Z-88.0)   # канал под стальную полосу 30x6 антипроворота, 2 x M6x20
TQ_PLATE_W = 30.0; TQ_PLATE_BELOW = 100.0; TQ_SLOT = 9.8; TQ_SLOT_DEPTH = 6.0   # пластина: ширина, длина ниже оси, паз под лыски, дно паза ниже оси
DO_AXLE_T = DO_T - TQ_DEPTH            # остаточная толщина вкладыша у оси (10)
SLOT_W = AXLE_FLAT + 0.1   # 9.8: подобрано по тесту 09 (1 риска), 1.11.2026
FOOT_X = (300.0, 370.0); FOOT_W = 60.0; FOOT_H = 14.0   # ступни боковин наружу (колея сзади ~297)
INNER_L = 105.0; HALF_IN = INNER_L/2   # фиксированный интерфейс заднего блока
CRANK_CHAINLINE = 43.5; HUB_CHAINLINE = 54.0   # условные, заменить замерами
HUB_Y = CRANK_CHAINLINE - HUB_CHAINLINE
SHIM_LEFT = (OLD-INNER_L)/2 - HUB_Y
SHIM_RIGHT = (OLD-INNER_L)/2 + HUB_Y
assert min(SHIM_LEFT, SHIM_RIGHT) >= 2.0 and max(SHIM_LEFT, SHIM_RIGHT) <= 40.0, "Chainline outside spacer range"
# Задний блок (поперечина + мост распорки слиты в одну деталь)
RC_X = (HUB_X-125.0, HUB_X-70.0); RC_H = 60.0
PB_X = (HUB_X-112.0, HUB_X-83.0); PB_Z = 50.0
BR_GAP = 8.0; BR_REAR_X = HUB_X - HUB_D/2 - BR_GAP
BR_Z = (150.0, 200.0); BR_B = (BR_REAR_X - 27.0, 178.0)
STRUT_T = (35.0, BB_Z - 30.0); STRUT_P1_X = 70.0; STRUT_HOLE = 5.75
BR_BOLTS = [(BR_REAR_X-19.0, 160.0, 'rear'), (BR_REAR_X-11.0, 191.0, 'up')]   # M8 мост->боковины, выход кармана гайки
WEB_X = (RC_X[0], BR_REAR_X); WEB_Y = (-HALF_IN, -HALF_IN+20.0)              # перемычка слева (сторона стола при печати)
CLAMP_X = (RC_X[0]+RC_X[1])/2; CLAMP_NUT_Z = 38.0                             # вертикальный M6x30 прижимает трубу
PEDAL_R = 215.0; SEG = 128
_d = Vector((BR_B[0]-STRUT_T[0], 0, BR_B[1]-STRUT_T[1])).normalized()
STRUT_U = _d; STRUT_N = Vector((-_d.z, 0, _d.x)); STRUT_B = Vector((BR_B[0], 0, BR_B[1]))
STRUT_P1 = STRUT_B - STRUT_U*((BR_B[0]-STRUT_P1_X)/STRUT_U.x)
STRUT_D = (STRUT_B - STRUT_P1).length
def xf(z): return BR_B[0] + (z-BR_B[1])*STRUT_N.x/STRUT_N.z

# ------------------------- ПОМОЩНИКИ -------------------------
def get_coll(name):
    c = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    c.hide_viewport = False; c.hide_render = False
    if c.name not in [x.name for x in bpy.context.scene.collection.children]:
        bpy.context.scene.collection.children.link(c)
    return c

def new_obj(name, bm, coll):
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    ob = bpy.data.objects.new(name, me); coll.objects.link(ob); return ob

def box(name, x0, x1, y0, y1, z0, z1, coll):
    bm = bmesh.new(); bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = Vector(((x0+x1)/2 + v.co.x*(x1-x0), (y0+y1)/2 + v.co.y*(y1-y0), (z0+z1)/2 + v.co.z*(z1-z0)))
    return new_obj(name, bm, coll)

def cyl(name, r, p0, p1, coll, seg=SEG, r2=None, rot=0.0):
    p0, p1 = Vector(p0), Vector(p1); d = p1 - p0
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=seg, radius1=r, radius2=(r if r2 is None else r2), depth=d.length)
    q = Vector((0, 0, 1)).rotation_difference(d.normalized())
    bm.transform(Matrix.Translation((p0 + p1)/2) @ q.to_matrix().to_4x4() @ Matrix.Rotation(rot, 4, 'Z'))
    return new_obj(name, bm, coll)

def prism(name, pts, a0, a1, plane, coll):
    bm = bmesh.new()
    if plane == 'XZ': V = lambda p, a: (p[0], a, p[1])
    elif plane == 'YZ': V = lambda p, a: (a, p[0], p[1])
    else: V = lambda p, a: (p[0], p[1], a)
    b = [bm.verts.new(V(p, a0)) for p in pts]; t = [bm.verts.new(V(p, a1)) for p in pts]
    bm.faces.new(b[::-1]); bm.faces.new(t)
    for i in range(len(pts)):
        j = (i+1) % len(pts); bm.faces.new((b[i], b[j], t[j], t[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return new_obj(name, bm, coll)

def tear(name, axis, c1, c2, r, a0, a1, phi, coll, seg=48):
    """Каплевидное отверстие вдоль axis ('X' или 'Y') — печатается без поддержек.
    (c1,c2) — центр в плоскости профиля (YZ для X, XZ для Y); phi — направление
    вершины капли в градусах в этой плоскости (90 = +Z, 0 = +второй оси); a0..a1 — длина."""
    pts = []
    for i in range(seg+1):
        a = math.radians(phi + 45 + 270.0*i/seg)
        pts.append((c1 + r*math.cos(a), c2 + r*math.sin(a)))
    k = r*math.sqrt(2)
    pts.append((c1 + k*math.cos(math.radians(phi)), c2 + k*math.sin(math.radians(phi))))
    return prism(name, pts, a0, a1, 'YZ' if axis == 'X' else 'XZ', coll)

def boolean(target, cutters, op):
    if not isinstance(cutters, (list, tuple)): cutters = [cutters]
    for c in cutters:
        m = target.modifiers.new("b", 'BOOLEAN'); m.operation = op; m.object = c
        try: m.solver = 'MANIFOLD'
        except Exception: m.solver = 'EXACT'
        dg = bpy.context.evaluated_depsgraph_get()
        me = bpy.data.meshes.new_from_object(target.evaluated_get(dg))
        old = target.data; target.modifiers.clear(); target.data = me; bpy.data.meshes.remove(old)
        cm = c.data; bpy.data.objects.remove(c); bpy.data.meshes.remove(cm)
    return target

def obox(name, p0, u, L, n, hn, y0, y1, coll):
    bm = bmesh.new(); bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        a = (v.co.x+0.5)*L; b = v.co.z*2*hn; y = y0 + (v.co.y+0.5)*(y1-y0)
        w = Vector(p0) + u*a + n*b; v.co = Vector((w.x, y, w.z))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return new_obj(name, bm, coll)

def hexprism(name, af, p0, p1, coll, rot=math.radians(30)):
    return cyl(name, af/2/math.cos(math.radians(30)), p0, p1, coll, seg=6, rot=rot)

def annulus(name, r_in, r_out, cx, y0, y1, cz, coll):
    o = cyl(name, r_out, (cx, y0, cz), (cx, y1, cz), coll)
    return boolean(o, cyl("in", r_in, (cx, y0-1, cz), (cx, y1+1, cz), coll), 'DIFFERENCE')

def mat(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name); m.diffuse_color = rgba; return m

def clean_mesh(ob):
    bm = bmesh.new(); bm.from_mesh(ob.data)
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=0.0001)
    bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=0.0001)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(ob.data); bm.free(); ob.data.update()

def copy_transformed(src, name, M, coll):
    me = src.data.copy(); me.transform(M); me.name = name
    ob = bpy.data.objects.new(name, me); coll.objects.link(ob); return ob

def setmat(ob, m):
    ob.data.materials.clear(); ob.data.materials.append(m); return ob

sc = bpy.context.scene
sc.unit_settings.system = 'METRIC'; sc.unit_settings.scale_length = 0.001; sc.unit_settings.length_unit = 'MILLIMETERS'
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
for me in list(bpy.data.meshes):
    if me.users == 0: bpy.data.meshes.remove(me)

C_PRINT = get_coll("1_Станок_печать"); C_TPL = get_coll("2_Тесты_печать"); C_REF = get_coll("3_Референс_не_печатать"); C_ENV = get_coll("4_Огибающие_проверка")
M_PETG = mat("PETG", (0.15, 0.45, 0.85, 1)); M_TPL = mat("PETG_тест", (0.95, 0.6, 0.1, 1))
M_INSERT = mat("PETG_сменный_вкладыш", (0.95, 0.48, 0.08, 1)); M_SHIM = mat("PETG_проставка", (0.3, 0.75, 0.35, 1))
M_STEEL = mat("Сталь", (0.6, 0.6, 0.62, 1)); M_HUB = mat("Мотор", (0.08, 0.08, 0.08, 1)); M_ALU = mat("Алюминий", (0.8, 0.8, 0.82, 1))
M_ENV = mat("Огибающая", (0.9, 0.2, 0.2, 0.3))

PARTS = {}   # имя файла -> (объект, матрица ориентации печати)
RX90 = Matrix.Rotation(math.radians(90), 4, 'X')    # +Y -> вверх (деталь лежит на грани -Y)
RXm90 = Matrix.Rotation(math.radians(-90), 4, 'X')  # -Y -> вверх (деталь лежит на грани +Y)
ID4 = Matrix.Identity(4)

# ===== 1. Башня =====
hy = TOWER_T/2
tower = prism("tower", [(-100, BAR_Z0), (100, BAR_Z0), (100, BAR_Z1), (-100, BAR_Z1)], -hy, hy, 'XZ', C_PRINT)
legF = prism("legF", [(-100, BAR_Z0), (-56, BAR_Z0), (-6, BB_Z+8), (-42, BB_Z-16)], -hy, hy, 'XZ', C_PRINT)
legR = prism("legR", [(56, BAR_Z0), (100, BAR_Z0), (42, BB_Z-16), (6, BB_Z+8)], -hy, hy, 'XZ', C_PRINT)
ring = cyl("ring", CLAMP_OD/2, (0, -hy, BB_Z), (0, hy, BB_Z), C_PRINT)
ears = box("ears", -16, 16, -hy, hy, BB_Z+26, BB_Z+50, C_PRINT)
boss = obox("boss", STRUT_P1 - STRUT_U*44, STRUT_U, 44, STRUT_N, 16, -hy, hy, C_PRINT)
boolean(tower, [legF, legR, ring, ears, boss], 'UNION')
cut = [cyl("bore", CLAMP_ID/2, (0, -hy-1, BB_Z), (0, hy+1, BB_Z), C_PRINT),
       box("slit", -1.5, 1.5, -hy-1, hy+1, BB_Z, BB_Z+60, C_PRINT)]
for yy in (-11, 11):
    cut.append(cyl("pinch", 3.3, (-20, yy, BB_Z+40), (20, yy, BB_Z+40), C_PRINT, seg=48))
for sx in (-1, 1):
    for z in TOWER_BOLT_Z:
        cut.append(cyl("sb", 4.3, (sx*TOWER_BOLT_X, -hy-1, z), (sx*TOWER_BOLT_X, hy+1, z), C_PRINT, seg=48))
cut.append(cyl("strut_hole", STRUT_HOLE, STRUT_P1 + STRUT_U*3, STRUT_P1 - STRUT_U*40, C_PRINT, seg=48))
boolean(tower, cut, 'DIFFERENCE'); setmat(tower, M_PETG)
PARTS["01_PETG_tower_x1"] = (tower, RX90)

# ===== 2. Гильза BB86 =====
sleeve = cyl("sleeve_bb86", SLEEVE_OD/2, (0, -SLEEVE_L/2, BB_Z), (0, SLEEVE_L/2, BB_Z), C_PRINT)
sc_cut = [cyl("sb", SLEEVE_ID/2, (0, -SLEEVE_L/2-1, BB_Z), (0, SLEEVE_L/2+1, BB_Z), C_PRINT)]
for s in (-1, 1):
    sc_cut.append(cyl("ch", SLEEVE_ID/2+1.2, (0, s*(SLEEVE_L/2+0.01), BB_Z), (0, s*(SLEEVE_L/2-1.2), BB_Z), C_PRINT, r2=SLEEVE_ID/2-0.01))
boolean(sleeve, sc_cut, 'DIFFERENCE'); setmat(sleeve, M_PETG)
PARTS["02_PETG_bb86_sleeve_x1"] = (sleeve, RX90)

# ===== 3. Половина основания (правая; левая = та же деталь, повёрнутая на 180°) =====
def base_half():
    b = box("base_half_R", -100, 100, 0, SPINE_Y, 0, SPINE_H, C_PRINT)
    adds = []
    for sx in (-1, 1):
        xa, xb = sorted((sx*PIER_X[0], sx*PIER_X[1]))
        adds.append(box("pier", xa, xb, 0, SPINE_Y, 0, PIER_H, C_PRINT))
        adds.append(box("wall", xa, xb, WALL_Y[0], WALL_Y[1], PIER_H-1, WALL_TOP, C_PRINT))
        xa, xb = sorted((sx*ARM_X[0], sx*ARM_X[1]))
        adds.append(prism("arm", [(ARM_Y0, 0), (BASE_W, 0), (BASE_W, ARM_H1), (SPINE_Y, ARM_H0), (ARM_Y0, ARM_H0)], xa, xb, 'YZ', C_PRINT))
    boolean(b, adds, 'UNION')
    c = [tear("tube", 'X', TUBE_Y, TUBE_Z, TUBE_HOLE/2, -101, 101, 90, C_PRINT, seg=64)]
    for sx in (-1, 1):
        for z in TOWER_BOLT_Z:
            c.append(tear("sb", 'Y', sx*TOWER_BOLT_X, z, 4.3, WALL_Y[0]-1, WALL_Y[1]+1, 90, C_PRINT))
            if sx > 0:   # гайка M8 в шестигранном кармане глубиной 7 (вершина шестигранника вверх при печати)
                c.append(hexprism("tbn", 13.4, (sx*TOWER_BOLT_X, WALL_Y[1]-7.0, z), (sx*TOWER_BOLT_X, WALL_Y[1]+1, z), C_PRINT))
            else:        # головка DIN 912 + шайба M8 в зенковке 16.5 глубиной 10
                c.append(cyl("tbh", 8.25, (sx*TOWER_BOLT_X, WALL_Y[1]-10.0, z), (sx*TOWER_BOLT_X, WALL_Y[1]+1, z), C_PRINT, seg=64))
        xa, xb = sorted((sx*STRAP_X[0], sx*STRAP_X[1]))
        c.append(box("strap", xa, xb, STRAP_Y[0], STRAP_Y[1], -1, ARM_H0+1, C_PRINT))
    c.append(tear("g6", 'Y', GRUB_X, TUBE_Z, 3.3, TUBE_Y+TUBE_HOLE/2-1, SPINE_Y+1, 90, C_PRINT, seg=32))
    c.append(hexprism("g6n", 10.3, (GRUB_X, GRUB_NUT_Y0, TUBE_Z), (GRUB_X, GRUB_NUT_Y0+5.5, TUBE_Z), C_PRINT))
    c.append(box("g6s", GRUB_X-5.15, GRUB_X+5.15, GRUB_NUT_Y0, GRUB_NUT_Y0+5.5, TUBE_Z, SPINE_H+1, C_PRINT))
    boolean(b, c, 'DIFFERENCE'); setmat(b, M_PETG); return b
baseR = base_half()
baseL = copy_transformed(baseR, "base_half_L", Matrix.Rotation(math.pi, 4, 'Z'), C_PRINT)
PARTS["03_PETG_base_half_x2"] = (baseR, ID4)

# ===== 4. Задний блок: поперечина + мост распорки + перемычка (одна деталь) =====
rb = box("rear_block", RC_X[0], RC_X[1], -HALF_IN, HALF_IN, 0, RC_H, C_PRINT)
bridge = prism("bridge", [(xf(BR_Z[0]), BR_Z[0]), (BR_REAR_X, BR_Z[0]), (BR_REAR_X, BR_Z[1]), (xf(BR_Z[1]), BR_Z[1])], -HALF_IN, HALF_IN, 'XZ', C_PRINT)
web = box("web", WEB_X[0], WEB_X[1], WEB_Y[0], WEB_Y[1], RC_H-1, BR_Z[0]+1, C_PRINT)
boolean(rb, [bridge, web], 'UNION')
c = [cyl("strut_hole", STRUT_HOLE, STRUT_B - STRUT_U*3, STRUT_B + STRUT_U*24, C_PRINT, seg=48)]
for sy in (-1, 1):
    c.append(tear("tube", 'X', sy*TUBE_Y, TUBE_Z, TUBE_HOLE/2, RC_X[0]-1, RC_X[1]+1, 0, C_PRINT, seg=64))   # вершина капли -> +Y (верх при печати на боку)
    c.append(cyl("cs", 3.3, (CLAMP_X, sy*TUBE_Y, TUBE_Z+8), (CLAMP_X, sy*TUBE_Y, RC_H+1), C_PRINT, seg=32))
    c.append(hexprism("csn", 10.3, (CLAMP_X, sy*TUBE_Y, CLAMP_NUT_Z), (CLAMP_X, sy*TUBE_Y, CLAMP_NUT_Z+5.5), C_PRINT, rot=0.0))
    c.append(box("css", CLAMP_X, RC_X[1]+1, sy*TUBE_Y-5.15, sy*TUBE_Y+5.15, CLAMP_NUT_Z, CLAMP_NUT_Z+5.5, C_PRINT))
    for x in PB_X:
        c.append(cyl("pb", 4.3, (x, sy*(HALF_IN+1), PB_Z), (x, sy*(HALF_IN-30), PB_Z), C_PRINT, seg=48))
        ya, yb = sorted((sy*(HALF_IN-8), sy*(HALF_IN-15)))
        c.append(box("nslot", x-6.8, x+6.8, ya, yb, PB_Z-7.9, RC_H+1, C_PRINT))
    for (bx, bz, sd) in BR_BOLTS:
        c.append(cyl("bh", 4.3, (bx, sy*(HALF_IN+1), bz), (bx, sy*(HALF_IN-30), bz), C_PRINT, seg=48))
        ya, yb = sorted((sy*(HALF_IN-8), sy*(HALF_IN-15)))
        if sd == 'up':
            c.append(hexprism("nut", 13.4, (bx, sy*(HALF_IN-8), bz), (bx, sy*(HALF_IN-15), bz), C_PRINT))
            c.append(box("ns", bx-6.7, bx+6.7, ya, yb, bz, BR_Z[1]+1, C_PRINT))
        else:   # карман открыт назад (к мотору): перемычка снизу не мешает
            c.append(hexprism("nut", 13.4, (bx, sy*(HALF_IN-8), bz), (bx, sy*(HALF_IN-15), bz), C_PRINT, rot=0.0))
            c.append(box("ns", bx, BR_REAR_X+1, ya, yb, bz-6.7, bz+6.7, C_PRINT))
boolean(rb, c, 'DIFFERENCE'); setmat(rb, M_PETG)
PARTS["04_PETG_rear_block_x1"] = (rb, RX90)

# ===== 5. Боковины + сменные вкладыши оси =====
def dropout(name, side):
    yi = OLD/2
    def Y(a, b): return (a+HUB_Y, b+HUB_Y) if side > 0 else (-b+HUB_Y, -a+HUB_Y)
    def yy(v): return HUB_Y + side*v
    poly = [(RC_X[0]-5, 0), (HUB_X+45, 0), (HUB_X+45, DO_H), (221, DO_H), (208, 150), (RC_X[0]-5, 64)]
    p = prism(name, poly, *Y(yi, yi+DO_T), 'XZ', C_PRINT)
    foot = box("foot", FOOT_X[0], FOOT_X[1], *Y(yi+DO_T-1, yi+DO_T+FOOT_W), 0, FOOT_H, C_PRINT)
    boolean(p, foot, 'UNION')
    c = [box("insert_core_pocket", HUB_X-18.2, HUB_X+18.2, *Y(yi-1, yi+DO_T+1), HUB_Z-27.2, DO_H+1, C_PRINT),
         box("insert_flange_pocket", HUB_X-32.2, HUB_X+32.2, *Y(yi+8, yi+DO_T+1), HUB_Z-47.2, DO_H+1, C_PRINT)]
    insert = box("axle_insert_"+name, HUB_X-18, HUB_X+18, *Y(yi, yi+DO_T), HUB_Z-27, DO_H, C_PRINT)
    boolean(insert, box("fl", HUB_X-32, HUB_X+32, *Y(yi+8, yi+DO_T), HUB_Z-47, DO_H, C_PRINT), 'UNION')
    ic = [cyl("axle", AXLE_D/2+0.15, (HUB_X, yy(yi-1), HUB_Z), (HUB_X, yy(yi+DO_T+1), HUB_Z), C_PRINT, seg=64),
          box("slot", HUB_X-SLOT_W/2, HUB_X+SLOT_W/2, *Y(yi-1, yi+DO_T+1), HUB_Z, DO_H+1, C_PRINT),
          prism("fun", [(HUB_X-SLOT_W/2-4, DO_H+0.01), (HUB_X-SLOT_W/2, DO_H-4), (HUB_X+SLOT_W/2, DO_H-4), (HUB_X+SLOT_W/2+4, DO_H+0.01)],
                *Y(yi-1, yi+DO_T+1), 'XZ', C_PRINT)]
    for dx in (-25, 25):
        for zz in (HUB_Z-37, HUB_Z+23):
            for cuts in (c, ic):
                cuts.append(cyl("m5", 2.75, (HUB_X+dx, yy(yi-1), zz), (HUB_X+dx, yy(yi+DO_T+1), zz), C_PRINT, seg=48))
            c.append(hexprism("m5n", 8.3, (HUB_X+dx, yy(yi-1), zz), (HUB_X+dx, yy(yi+4.2), zz), C_PRINT))
    # Канал под стальную полосу антипроворота 31 x 6 мм с наружной стороны, в боковине и во вкладыше
    for cuts in (c, ic):
        cuts.append(box("tq", HUB_X-TQ_W/2, HUB_X+TQ_W/2, *Y(yi+DO_AXLE_T, yi+DO_T+1), TQ_Z0, HUB_Z, C_PRINT))
        cuts.append(cyl("tqr", TQ_W/2, (HUB_X, yy(yi+DO_AXLE_T), HUB_Z), (HUB_X, yy(yi+DO_T+1), HUB_Z), C_PRINT, seg=96))
    for zz in TQ_BOLT_Z:
        c.append(cyl("tqb", 3.3, (HUB_X, yy(yi-1), zz), (HUB_X, yy(yi+DO_T+1), zz), C_PRINT, seg=48))
        c.append(hexprism("tqn", 10.3, (HUB_X, yy(yi-1), zz), (HUB_X, yy(yi+5.2), zz), C_PRINT))
    boolean(insert, ic, 'DIFFERENCE'); setmat(insert, M_INSERT)
    for x in PB_X:
        c.append(cyl("pb", 4.3, (x, yy(yi-1), PB_Z), (x, yy(yi+DO_T+1), PB_Z), C_PRINT, seg=48))
    for (bx, bz, _) in BR_BOLTS:
        c.append(cyl("brb", 4.3, (bx, yy(yi-1), bz), (bx, yy(yi+DO_T+1), bz), C_PRINT, seg=48))
    boolean(p, c, 'DIFFERENCE'); setmat(p, M_PETG); return p, insert
doR, insR = dropout("dropout_right", 1); doL, insL = dropout("dropout_left", -1)
PARTS["05_PETG_dropout_right_x1"] = (doR, RX90)
PARTS["06_PETG_dropout_left_x1"] = (doL, RXm90)
PARTS["08_PETG_axle_insert_x2"] = (insR, RXm90)   # печать плашмя наружной стороной (канал под пластину) на столе: нагрузки от оси в плоскости слоёв; поддержка только в канале

# ===== 6. Проставки чейнлайна: одна цельная на сторону (нижняя + верхняя площадки + перемычка) =====
def spacer(side, t):
    sgn = 1 if side == "right" else -1
    y0, y1 = sorted((sgn*HALF_IN, sgn*(HALF_IN+t)))
    lo = box("spacer_"+side, RC_X[0], RC_X[1], y0, y1, 30, RC_H, C_PRINT)
    hi = prism("sp_hi", [(xf(BR_Z[0]), BR_Z[0]), (BR_REAR_X, BR_Z[0]), (BR_REAR_X, BR_Z[1]), (xf(BR_Z[1]), BR_Z[1])], y0, y1, 'XZ', C_PRINT)
    wb = box("sp_web", BR_REAR_X-10, BR_REAR_X, y0, y1, RC_H-1, BR_Z[0]+1, C_PRINT)
    boolean(lo, [hi, wb], 'UNION')
    c = [cyl("h", 4.4, (x, y0-1, PB_Z), (x, y1+1, PB_Z), C_PRINT, seg=48) for x in PB_X]
    c += [cyl("h", 4.4, (bx, y0-1, bz), (bx, y1+1, bz), C_PRINT, seg=48) for (bx, bz, _) in BR_BOLTS]
    for k in range(1 if side == "left" else 2):   # риски: одна = левая, две = правая
        c.append(box("m", RC_X[0]+4+k*4, RC_X[0]+6+k*4, y0-1, y1+1, 29, 33, C_PRINT))
    boolean(lo, c, 'DIFFERENCE'); setmat(lo, M_SHIM); return lo
spL = spacer("left", SHIM_LEFT); spR = spacer("right", SHIM_RIGHT)
PARTS["07_PETG_spacer_left_%smm_x1" % format(SHIM_LEFT, "g").replace(".", "p")] = (spL, RX90)
PARTS["07_PETG_spacer_right_%smm_x1" % format(SHIM_RIGHT, "g").replace(".", "p")] = (spR, RXm90)

# ===== 7. Тесты посадок (печатать первыми) =====
tc = box("tpl_dropout_slot", 0, 75, 0, 34, 0, 10, C_TPL)
c = []
for i, w in enumerate((AXLE_FLAT+0.1, AXLE_FLAT+0.3, AXLE_FLAT+0.5)):
    x = 14 + i*24
    c.append(box("s", x-w/2, x+w/2, 14, 35, -1, 11, C_TPL))
    c.append(cyl("h", AXLE_D/2+0.15, (x, 14, -1), (x, 14, 11), C_TPL, seg=64))
    for k in range(i+1): c.append(box("m", x-4+k*3, x-3+k*3, -1, 2, 8, 11, C_TPL))
boolean(tc, c, 'DIFFERENCE'); tc.location = (-250, 380, 0); setmat(tc, M_TPL)
PARTS["09_PETG_TEST_dropout_slot_x1"] = (tc, ID4)
for i, d in enumerate((40.9, 41.1, 41.3, 41.5)):
    tag = str(d).replace('.', 'p'); x = (i % 2)*62; yb = (i // 2)*62
    r = cyl("tpl_bb86_ring_ID"+tag, 27.5, (x, yb, 0), (x, yb, 12), C_TPL)
    cc = [cyl("in", d/2, (x, yb, -1), (x, yb, 13), C_TPL), cyl("ch", d/2+1.0, (x, yb, 12.01), (x, yb, 10.99), C_TPL, r2=d/2-0.01)]
    for k in range(i+1):
        xm = x - 6*i/2 + 6*k
        cc.append(box("m", xm-1, xm+1, yb+24, yb+29, 9, 13, C_TPL))
    boolean(r, cc, 'DIFFERENCE'); r.location = (-250, 480, 0); setmat(r, M_TPL)
    PARTS["10_PETG_TEST_bb86_ring_ID%s_x1" % tag] = (r, ID4)

# ===== 8. Референс (металл, мотор, привод) — не печатать =====
for s in (-1, 1):
    r = cyl("REF_alu_tube_%dx%d" % (TUBE_D, TUBE_LEN), TUBE_D/2, (TUBE_X0, s*TUBE_Y, TUBE_Z), (TUBE_X0+TUBE_LEN, s*TUBE_Y, TUBE_Z), C_REF, seg=48)
    boolean(r, cyl("bore", TUBE_D/2-TUBE_WALL, (TUBE_X0-1, s*TUBE_Y, TUBE_Z), (TUBE_X0+TUBE_LEN+1, s*TUBE_Y, TUBE_Z), C_REF, seg=48), 'DIFFERENCE')
    setmat(r, M_ALU)
STRUT_ROD_LEN = 200; STRUT_FRONT_INSERT = STRUT_ROD_LEN - STRUT_D - 22.0
sr = cyl("REF_strut_rod_M10x%d" % STRUT_ROD_LEN, 5, STRUT_P1 - STRUT_U*STRUT_FRONT_INSERT, STRUT_P1 - STRUT_U*STRUT_FRONT_INSERT + STRUT_U*STRUT_ROD_LEN, C_REF, seg=32)
setmat(sr, M_STEEL)
for face, direction in ((STRUT_P1, 1), (STRUT_B, -1)):
    u = STRUT_U*direction
    w = cyl("REF_strut_washer_M10", 15, face, face+u*2.5, C_REF)
    boolean(w, cyl("bore", 5.5, face-u, face+u*3.5, C_REF), 'DIFFERENCE'); setmat(w, M_STEEL)
    for distance in (2.5, 10.5):
        q = face+u*distance
        setmat(hexprism("REF_strut_nut_M10", 17, q, q+u*8, C_REF), M_STEEL)
def at(y): return (HUB_X, y+HUB_Y, HUB_Z)
hub = cyl("REF_hub_motor", HUB_ACTUAL_D/2, at(-HUB_W/2), at(HUB_W/2), C_REF)
boolean(hub, [cyl("capL", HUB_CAP_D/2, at(-HUB_BODY_W/2), at(-HUB_W/2+0.1), C_REF),
              cyl("capR", HUB_CAP_D/2, at(HUB_W/2-0.1), at(HUB_BODY_W/2), C_REF),
              cyl("shL", HUB_SHOULDER_D/2, at(-OLD/2), at(-HUB_BODY_W/2+0.1), C_REF, seg=64),
              cyl("shR", HUB_SHOULDER_D/2, at(HUB_BODY_W/2-0.1), at(OLD/2), C_REF, seg=64)], 'UNION')
setmat(hub, M_HUB)
ax = cyl("REF_axle", AXLE_D/2, at(-AXLE_LEN/2), at(AXLE_LEN/2), C_REF, seg=64)
boolean(ax, [box("f", HUB_X+AXLE_FLAT/2, HUB_X+AXLE_D, -AXLE_LEN, AXLE_LEN, HUB_Z-AXLE_D, HUB_Z+AXLE_D, C_REF),
             box("f", HUB_X-AXLE_D, HUB_X-AXLE_FLAT/2, -AXLE_LEN, AXLE_LEN, HUB_Z-AXLE_D, HUB_Z+AXLE_D, C_REF)], 'DIFFERENCE')
setmat(ax, M_STEEL)
fw = annulus("REF_disc_mount_sprocket_16T_ASSUMED", 17.5, 33, HUB_X, HUB_Y+HUB_CHAINLINE-1.5, HUB_Y+HUB_CHAINLINE+1.5, HUB_Z, C_REF); setmat(fw, M_STEEL)
setmat(cyl("REF_cable_exit_left_ASSUMED", 3, at(-AXLE_LEN/2), at(-AXLE_LEN/2-20), C_REF, seg=24), M_HUB)
# Ось: стальная пластина антипроворота (полоса 30x6 в канале боковины), штатные шайба и гайка снаружи
TQ_REF = []; TQB_REF = []
for s in (-1, 1):
    y_in = HUB_Y + s*(OLD/2 + DO_AXLE_T); y_out = HUB_Y + s*(OLD/2 + DO_T)
    ya, yb = sorted((y_in, y_out))
    tq = box("REF_torque_plate_steel_30x6", HUB_X-TQ_PLATE_W/2, HUB_X+TQ_PLATE_W/2, ya, yb, HUB_Z-TQ_PLATE_BELOW, HUB_Z, C_REF)
    boolean(tq, cyl("tqtop", TQ_PLATE_W/2, (HUB_X, ya, HUB_Z), (HUB_X, yb, HUB_Z), C_REF, seg=96), 'UNION')
    boolean(tq, [box("slot", HUB_X-TQ_SLOT/2, HUB_X+TQ_SLOT/2, ya-1, yb+1, HUB_Z-TQ_SLOT_DEPTH, HUB_Z+30, C_REF)] +
                [cyl("h", 3.3, (HUB_X, ya-1, z), (HUB_X, yb+1, z), C_REF, seg=24) for z in TQ_BOLT_Z], 'DIFFERENCE')
    setmat(tq, M_STEEL); TQ_REF.append(tq)
    setmat(cyl("REF_axle_washer_steel_3mm", 18.5, (HUB_X, y_out, HUB_Z), (HUB_X, y_out+s*3, HUB_Z), C_REF, seg=64), M_STEEL)
    setmat(hexprism("REF_axle_nut", 18, (HUB_X, y_out+s*3, HUB_Z), (HUB_X, y_out+s*11, HUB_Z), C_REF), M_STEEL)
    for z in TQ_BOLT_Z:
        tb = setmat(cyl("REF_tq_bolt_M6x20", 3, (HUB_X, y_out, z), (HUB_X, y_out-s*20, z), C_REF, seg=24), M_STEEL); TQB_REF.append(tb)
        setmat(cyl("REF_tq_bolt_head_DIN912", 5, (HUB_X, y_out, z), (HUB_X, y_out+s*6, z), C_REF, seg=24), M_STEEL)
PARTS["11_PETG_TEMPLATE_torque_plate_30x6_x2"] = (TQ_REF[1], RX90)   # пластиковый шаблон пластины: примерка, образец для мастерской, лёгкие тесты; силовая — сталь
# Болты боковин: M8x70 слева, M8x50 справа (референс стержней для проверки пересечений с распоркой)
BR_REF = []
for s, L in ((-1, 70), (1, 50)):
    y_head = HUB_Y + s*(OLD/2 + DO_T + 2)
    for (bx, bz) in [(x, PB_Z) for x in PB_X] + [(bx, bz) for (bx, bz, _) in BR_BOLTS]:
        b = cyl("REF_side_bolt_M8x%d" % L, 4, (bx, y_head, bz), (bx, y_head - s*L, bz), C_REF, seg=24); setmat(b, M_STEEL); BR_REF.append(b)
# Болты башни: 4 x M8x80 DIN 912 (головка в зенковке, гайка в кармане)
TB_REF = []
for sx in (-1, 1):
    for z in TOWER_BOLT_Z:
        x = sx*TOWER_BOLT_X; s = 1 if sx > 0 else -1; y_seat = -s*(WALL_Y[1]-10.0+1.6)
        b = cyl("REF_tower_bolt_M8x80", 4, (x, y_seat, z), (x, y_seat+s*80, z), C_REF, seg=24); setmat(b, M_STEEL); TB_REF.append(b)
        TB_REF.append(setmat(cyl("REF_tower_bolt_head_DIN912", 6.5, (x, y_seat, z), (x, y_seat-s*8, z), C_REF, seg=32), M_STEEL))
        TB_REF.append(setmat(hexprism("REF_tower_nut_M8", 13, (x, s*(WALL_Y[1]-7.0), z), (x, s*(WALL_Y[1]-0.5), z), C_REF), M_STEEL))
        setmat(cyl("REF_washer_M8", 8.0, (x, -s*(WALL_Y[1]-10.0), z), (x, y_seat, z), C_REF, seg=32), M_STEEL)
cr = annulus("REF_chainring_44T", 50, 90, 0, CRANK_CHAINLINE-2, CRANK_CHAINLINE+2, BB_Z, C_REF); setmat(cr, M_ALU)
setmat(cyl("REF_spindle_24", 12, (0, -70, BB_Z), (0, 70, BB_Z), C_REF, seg=48), M_STEEL)
for s, ang in ((1, math.radians(-90)), (-1, math.radians(90))):
    tip = (170*math.cos(ang), s*62, BB_Z + 170*math.sin(ang))
    setmat(cyl("REF_crank_170", 11, (0, s*62, BB_Z), tip, C_REF, seg=24), M_HUB)
    setmat(box("REF_pedal", tip[0]-35, tip[0]+35, *((75, 165) if s > 0 else (-165, -75)), tip[2]-12, tip[2]+12, C_REF), M_HUB)
chain_r1 = 12.7/(2*math.sin(math.pi/44)); chain_r2 = 12.7/(2*math.sin(math.pi/16))
dx, dz = HUB_X, HUB_Z-BB_Z; centre_distance = math.hypot(dx, dz)
ux, uz = dx/centre_distance, dz/centre_distance; chain_a = (chain_r1-chain_r2)/centre_distance
CHAIN_REF = []; CHAIN_RUNS = {}
for sign in (-1, 1):
    q = math.sqrt(1-chain_a*chain_a); nx = chain_a*ux-sign*q*uz; nz = chain_a*uz+sign*q*ux
    p1 = (chain_r1*nx, BB_Z+chain_r1*nz); p2 = (HUB_X+chain_r2*nx, HUB_Z+chain_r2*nz)
    run = "lower" if nz < 0 else "upper"; CHAIN_RUNS[run] = (p1, p2)
    ch = cyl("REF_chain_"+run, 3, (p1[0], CRANK_CHAINLINE, p1[1]), (p2[0], CRANK_CHAINLINE, p2[1]), C_REF, seg=12)
    setmat(ch, M_STEEL); CHAIN_REF.append(ch)
def chain_z(run, x):
    (x1, z1), (x2, z2) = CHAIN_RUNS[run]; return z1 + (x-x1)*(z2-z1)/(x2-x1)
# Огибающие шатунов (диск R173.5, |Y| 48..76) и педалей (кольцо R133..207, |Y| 72..168): только для проверок
ENV = {}
for s in (-1, 1):
    tag = 'R' if s > 0 else 'L'
    y0, y1 = sorted((s*48, s*76))
    ENV["crank_"+tag] = setmat(cyl("ENV_crank_"+tag, 173.5, (0, y0, BB_Z), (0, y1, BB_Z), C_ENV), M_ENV)
    y0, y1 = sorted((s*72, s*168))
    ENV["pedal_"+tag] = setmat(annulus("ENV_pedal_"+tag, 133, 207, 0, y0, y1, BB_Z, C_ENV), M_ENV)

# ===== 9. Очистка сеток печатных деталей =====
for ob, _ in PARTS.values(): clean_mesh(ob)
clean_mesh(baseL)

# ===== 10. Проверки пересечений (точный булев INTERSECT, объём в мм³) =====
C_TMP = get_coll("_tmp")
def ivol(a, b):
    tmp = bpy.data.objects.new("tmp_int", a.data.copy()); tmp.matrix_world = a.matrix_world.copy(); C_TMP.objects.link(tmp)
    m = tmp.modifiers.new("i", 'BOOLEAN'); m.operation = 'INTERSECT'; m.object = b; m.solver = 'EXACT'
    dg = bpy.context.evaluated_depsgraph_get(); ev = tmp.evaluated_get(dg); me2 = ev.to_mesh()
    bm = bmesh.new(); bm.from_mesh(me2); vol = abs(bm.calc_volume()); bm.free(); ev.to_mesh_clear()
    md = tmp.data; bpy.data.objects.remove(tmp); bpy.data.meshes.remove(md)
    return round(vol, 3)
tubes = [o for o in C_REF.objects if o.name.startswith("REF_alu_tube")]
pairs = [("tower", tower, "base_R", baseR), ("tower", tower, "base_L", baseL), ("base_R", baseR, "base_L", baseL),
         ("sleeve", sleeve, "tower", tower), ("chainring", cr, "tower", tower), ("chainring", cr, "base_R", baseR), ("chainring", cr, "sleeve", sleeve),
         ("hub", hub, "rear_block", rb), ("hub", hub, "dropout_R", doR), ("hub", hub, "dropout_L", doL), ("hub", hub, "insert_R", insR),
         ("strut_rod", sr, "rear_block", rb), ("strut_rod", sr, "tower", tower),
         ("spacer_L", spL, "dropout_L", doL), ("spacer_L", spL, "rear_block", rb), ("spacer_R", spR, "dropout_R", doR), ("spacer_R", spR, "rear_block", rb),
         ("insert_R", insR, "dropout_R", doR), ("insert_L", insL, "dropout_L", doL), ("axle", ax, "insert_R", insR), ("axle", ax, "insert_L", insL),
         ("axle", ax, "dropout_R", doR), ("sprocket", fw, "dropout_R", doR), ("sprocket", fw, "insert_R", insR)]
for k, e in ENV.items():
    for n, o in (("base_R", baseR), ("base_L", baseL), ("tower", tower), ("sleeve", sleeve), ("rear_block", rb)):
        pairs.append((k, e, n, o))
for i, t in enumerate(tubes):
    for n, o in (("base_R", baseR), ("base_L", baseL), ("rear_block", rb)): pairs.append(("tube%d" % i, t, n, o))
for i, b in enumerate(TB_REF):
    for n, o in (("tower", tower), ("base_R", baseR), ("base_L", baseL)): pairs.append(("tower_bolt%d" % i, b, n, o))
for i, b in enumerate(BR_REF): pairs.append(("side_bolt%d" % i, b, "strut_rod", sr))
for i, b in enumerate(BR_REF): pairs.append(("side_bolt%d" % i, b, "rear_block", rb))
for i, t in enumerate(TQ_REF):
    for n, o in (("dropout_R", doR), ("dropout_L", doL), ("insert_R", insR), ("insert_L", insL)): pairs.append(("torque_plate%d" % i, t, n, o))
for i, t in enumerate(TQ_REF): pairs.append(("torque_plate%d" % i, t, "axle", ax))
for i, b in enumerate(TB_REF):
    for k, e in ENV.items(): pairs.append(("tower_hw%d" % i, b, k, e))
for i, b in enumerate(TQB_REF): pairs.append(("tq_bolt%d" % i, b, "hub", hub))
for k, e in ENV.items():
    for n, o in (("spacer_L", spL), ("spacer_R", spR), ("dropout_R", doR), ("dropout_L", doL)): pairs.append((k, e, n, o))
for i, ch in enumerate(CHAIN_REF):
    for n, o in (("rear_block", rb), ("tower", tower), ("dropout_R", doR), ("base_R", baseR)): pairs.append((ch.name, ch, n, o))
CHECKS = {"intersections_mm3": {}, "tolerance_mm3": 1.0, "violations": []}
for an, a, bn, b in pairs:
    v = ivol(a, b); CHECKS["intersections_mm3"]["%s|%s" % (an, bn)] = v
    if v > CHECKS["tolerance_mm3"]: CHECKS["violations"].append("%s|%s=%.1f" % (an, bn, v))
bpy.data.collections.remove(C_TMP)

# ===== 11. Экспорт одиночных STL (ориентация печати, Z вверх, низ на Z=0) =====
def write_stl(path, tris, header):
    with open(path, 'wb') as f:
        f.write(header.encode('ascii', 'ignore')[:80].ljust(80, b' ')); f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            n = (b-a).cross(c-a)
            if n.length < 1e-12: n = Vector((0, 0, 1))
            else: n.normalize()
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))
def mesh_stats(me):
    bm = bmesh.new(); bm.from_mesh(me)
    nm = sum(1 for e in bm.edges if not e.is_manifold); vol = bm.calc_volume()
    remaining = set(bm.verts); comps = 0
    while remaining:
        comps += 1; stack = [remaining.pop()]
        while stack:
            v = stack.pop()
            for e in v.link_edges:
                w = e.other_vert(v)
                if w in remaining: remaining.remove(w); stack.append(w)
    bm.free(); return nm, vol, comps
os.makedirs(STL_DIR, exist_ok=True)
for f in os.listdir(STL_DIR):
    if f.endswith('.stl'): os.remove(os.path.join(STL_DIR, f))
MANIFEST = []
for fname, (ob, M) in PARTS.items():
    me = ob.data.copy(); me.transform(ob.matrix_world); me.transform(M)
    xs = [v.co.x for v in me.vertices]; ys = [v.co.y for v in me.vertices]; zs = [v.co.z for v in me.vertices]
    me.transform(Matrix.Translation((-(min(xs)+max(xs))/2, -(min(ys)+max(ys))/2, -min(zs))))
    me.calc_loop_triangles()
    tris = [[me.vertices[i].co.copy() for i in t.vertices] for t in me.loop_triangles]
    write_stl(os.path.join(STL_DIR, fname + ".stl"), tris, "Bike generator stand v5 | mm | print Z up | " + fname)
    nm, vol, comps = mesh_stats(me)
    size = [round(max(xs)-min(xs), 2), round(max(ys)-min(ys), 2), round(max(zs)-min(zs), 2)]
    qty = int(fname.split("_x")[-1]); material = fname.split("_")[1]
    role = "test_fit" if "TEST" in fname else "template" if "TEMPLATE" in fname else ("chainline_spacer" if "spacer" in fname else ("replaceable_wear_part" if "insert" in fname else "structural"))
    MANIFEST.append({"file": fname + ".stl", "material": material, "quantity": qty, "role": role, "print_size_mm": size,
                     "fits_220x220x240": size[0] <= 220 and size[1] <= 220 and size[2] <= 240,
                     "non_manifold_edges": nm, "components": comps, "volume_cm3": round(abs(vol)/1000, 1), "triangles": len(tris)})
    if nm or comps != 1 or not MANIFEST[-1]["fits_220x220x240"]:
        CHECKS["violations"].append("part %s: non_manifold=%d components=%d size=%s" % (fname, nm, comps, size))
    bpy.data.meshes.remove(me)
with open(os.path.join(PROJECT, "print_manifest.json"), "w", encoding="utf-8") as f:
    json.dump({"units": "mm", "print_axis": "Z", "bed_mm": [220, 220, 240], "parts": MANIFEST}, f, ensure_ascii=False, indent=2)
with open(os.path.join(PROJECT, "print_manifest.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f); w.writerow(["file", "material", "quantity", "role", "print_size_mm", "volume_cm3"])
    for r in MANIFEST: w.writerow([r["file"], r["material"], r["quantity"], r["role"], " x ".join(map(str, r["print_size_mm"])), r["volume_cm3"]])

# ===== 12. Сборочный STL в корне версии (не для печати) =====
asm_objects = [o for o in C_PRINT.objects if o.type == 'MESH'] + [o for o in C_REF.objects if o.type == 'MESH']
asm_tris = []; asm_items = []
for ob in sorted(asm_objects, key=lambda o: o.name):
    me = ob.data; me.calc_loop_triangles(); start = len(asm_tris)
    for t in me.loop_triangles:
        asm_tris.append([ob.matrix_world @ me.vertices[i].co for i in t.vertices])
    asm_items.append({"object": ob.name, "triangles": len(asm_tris)-start})
write_stl(os.path.join(PROJECT, "bike_generator_stand_assembled.stl"), asm_tris, "Assembled reference v5 | millimetres | NOT FOR PRINTING")
allv = [v for t in asm_tris for v in t]
bounds = [[round(min(v[i] for v in allv), 1) for i in range(3)], [round(max(v[i] for v in allv), 1) for i in range(3)]]
with open(os.path.join(PROJECT, "assembly_stl_manifest.json"), "w", encoding="utf-8") as f:
    json.dump({"units": "mm", "purpose": "assembled_visual_reference_not_for_printing", "objects": asm_items, "triangles": len(asm_tris),
               "bounds_min_mm": bounds[0], "bounds_max_mm": bounds[1], "excluded": "test parts, envelopes, camera"}, f, ensure_ascii=False, indent=2)

# ===== 13. Отчёт сборки и проверок =====
printed_cm3 = sum(r["volume_cm3"]*r["quantity"] for r in MANIFEST if "TEST" not in r["file"] and "TEMPLATE" not in r["file"])
petg_mass_kg = printed_cm3*1.27*0.55/1000     # ~55% эффективной плотности (6-8 периметров + 30-40% заполнения)
hardware_kg = {"hub_motor_no_rim": 4.5, "cranks_chainring_bb": 1.3, "chain": 0.3, "alu_tubes_2x400": 0.25, "steel_fasteners_strut_torque_plates": 0.7}
mass_kg = petg_mass_kg + sum(hardware_kg.values())
g = 9.81; cg_x = 150.0; half_track = BASE_W; rear_edge_x = HUB_X + 45.0; top_pedal_z = BB_Z + 170
REPORT = {
    "version": "v5", "status": "HOLD_FOR_ACTUAL_DIMENSIONS_AND_FIT_TESTS",
    "params": {"BB_Z": BB_Z, "HUB_X": HUB_X, "HUB_Z": HUB_Z, "OLD": OLD, "AXLE_D": AXLE_D, "AXLE_FLAT": AXLE_FLAT, "DO_AXLE_T": DO_AXLE_T,
               "front_track_mm": 2*BASE_W, "rear_track_mm": round(2*(OLD/2+DO_T+FOOT_W), 1), "tube_length_mm": TUBE_LEN, "tube_x_range": [TUBE_X0, TUBE_X0+TUBE_LEN],
               "strut_length_mm": STRUT_ROD_LEN, "strut_bearing_distance_mm": round(STRUT_D, 1), "strut_front_insert_mm": round(STRUT_FRONT_INSERT, 1),
               "bb_to_hub_distance_mm": round(math.hypot(HUB_X, BB_Z-HUB_Z), 1), "chainline_front_mm": CRANK_CHAINLINE, "chainline_rear_mm": HUB_Y+HUB_CHAINLINE,
               "hub_offset_y_mm": HUB_Y, "spacer_left_mm": SHIM_LEFT, "spacer_right_mm": SHIM_RIGHT, "axle_stack_per_side_mm": DO_T + 3 + 8,
               "chain_lower_run_centre_z_mm": {"at_bridge_front_x%d" % int(xf(BR_Z[0])): round(chain_z("lower", xf(BR_Z[0])), 1), "at_bridge_rear_x%d" % int(BR_REAR_X): round(chain_z("lower", BR_REAR_X), 1)},
               "bridge_bottom_z_mm": BR_Z[0], "chain_upper_run_centre_z_at_bridge_rear_mm": round(chain_z("upper", BR_REAR_X), 1), "bridge_top_z_mm": BR_Z[1]},
    "torque_plate": {"material": "steel flat bar 30x6 (S235/S355 or AISI 304), 2 pcs", "length_mm": TQ_PLATE_BELOW + TQ_PLATE_W/2, "slot_mm": TQ_SLOT,
                     "slot_bottom_below_axle_mm": TQ_SLOT_DEPTH, "holes": "2 x 6.6 at %g and %g mm below axle" % (HUB_Z-TQ_BOLT_Z[0], HUB_Z-TQ_BOLT_Z[1]),
                     "bolts": "2 x M6x20 DIN 912 per side, nuts captive in dropout", "dxf": "torque_plate_steel_30x6_x2.dxf",
                     "hub_torque_100kg_on_horizontal_pedal_Nm": round(166.77*16/44, 1),
                     "force_on_two_M6_per_side_N": round(166.77*16/44/2/((2*HUB_Z-TQ_BOLT_Z[0]-TQ_BOLT_Z[1])/2000), 0)},
    "printed_volume_cm3": round(printed_cm3, 1), "petg_mass_estimate_kg": round(petg_mass_kg, 2), "hardware_mass_estimate_kg": hardware_kg,
    "total_mass_estimate_kg": round(mass_kg, 1),
    "stability_illustrative": {
        "method": "rigid body on floor, no anchoring; illustrative, not certification",
        "lateral_tip_force_N_at_top_pedal_no_vertical_load": round(mass_kg*g*half_track/1000/(top_pedal_z/1000), 1),
        "lateral_tip_force_N_at_top_pedal_with_300N_on_pedal_y110": round((mass_kg*g*half_track/1000 + 300*(half_track-110)/1000)/(top_pedal_z/1000), 1),
        "vertical_pedal_force_inside_footprint": "pedal |Y|~110 < half-track 200: vertical pedal load cannot tip the stand",
        "longitudinal_slide_force_N_mu0p4": round(0.4*mass_kg*g, 1),
        "longitudinal_tip_force_N_about_rear_edge": round(mass_kg*g*(rear_edge_x-cg_x)/1000/(top_pedal_z/1000), 1),
        "note": "horizontal pedal force at top of stroke at 200-500 W is ~100-300 N: tie the stand to the chair (strap slots in wings) or it will slide/rock"},
    "unresolved": ["actual axle thread length per side (>= %d mm needed beyond OLD plane)" % (DO_T + 3 + 8),
                   "exact OLD between contact planes", "actual front and rear chainline", "torque plate made to the real axle flats",
                   "disc-mount sprocket suitability", "fit tests of slot and BB86 sleeve", "slicer support check", "staged load test"]}
def plate_dxf(path):
    r = TQ_PLATE_W/2; hx = TQ_SLOT/2; a = math.degrees(math.acos(hx/r)); yt = r*math.sin(math.radians(a)); L = TQ_PLATE_BELOW; sd = TQ_SLOT_DEPTH
    ents = []
    def line(x1, y1, x2, y2): ents.append("0\nLINE\n8\n0\n10\n%.3f\n20\n%.3f\n30\n0\n11\n%.3f\n21\n%.3f\n31\n0" % (x1, y1, x2, y2))
    def arc(cx, cy, rr, a1, a2): ents.append("0\nARC\n8\n0\n10\n%.3f\n20\n%.3f\n30\n0\n40\n%.3f\n50\n%.3f\n51\n%.3f" % (cx, cy, rr, a1, a2))
    def circle(cx, cy, rr): ents.append("0\nCIRCLE\n8\n0\n10\n%.3f\n20\n%.3f\n30\n0\n40\n%.3f" % (cx, cy, rr))
    line(-r, -L, r, -L); line(r, -L, r, 0); arc(0, 0, r, 0, a); line(hx, yt, hx, -sd); line(hx, -sd, -hx, -sd); line(-hx, -sd, -hx, yt)
    arc(0, 0, r, 180-a, 180); line(-r, 0, -r, -L)
    for z in TQ_BOLT_Z: circle(0, z-HUB_Z, 3.3)
    with open(path, "w") as f: f.write("0\nSECTION\n2\nENTITIES\n" + "\n".join(ents) + "\n0\nENDSEC\n0\nEOF\n")
plate_dxf(os.path.join(PROJECT, "torque_plate_steel_30x6_x2.dxf"))
with open(os.path.join(PROJECT, "assembly_report.json"), "w", encoding="utf-8") as f: json.dump(REPORT, f, ensure_ascii=False, indent=2)
with open(os.path.join(PROJECT, "checks.json"), "w", encoding="utf-8") as f: json.dump(CHECKS, f, ensure_ascii=False, indent=2)

# ===== 14. Превью и сохранение =====
for o in C_TPL.objects: o.hide_render = True; o.hide_set(True) if bpy.context.view_layer.objects.get(o.name) else None
for o in C_ENV.objects: o.hide_render = True
C_ENV.hide_viewport = True
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = 1400; sc.render.resolution_y = 1000; sc.render.resolution_percentage = 100
sc.display.shading.light = 'STUDIO'; sc.display.shading.color_type = 'MATERIAL'
sc.display.shading.show_shadows = True; sc.display.shading.show_cavity = True; sc.display.shading.background_type = 'WORLD'
if sc.world is None: sc.world = bpy.data.worlds.new('World')
sc.world.color = (0.12, 0.12, 0.12); sc.render.film_transparent = False
def camera(name, loc, target, ortho):
    cd = bpy.data.cameras.new(name); cam = bpy.data.objects.new(name, cd); sc.collection.objects.link(cam)
    cam.location = loc; cam.rotation_euler = (Vector(target)-Vector(loc)).to_track_quat('-Z', 'Y').to_euler()
    cd.clip_end = 10000; cd.type = 'ORTHO'; cd.ortho_scale = ortho; return cam
cams = [("preview_assembly.png", camera("Cam_iso", (680, -1100, 720), (130, 0, 150), 820)),
        ("preview_front.png", camera("Cam_front", (-1200, -400, 350), (130, 0, 150), 720)),
        ("preview_rear.png", camera("Cam_rear", (1100, 700, 520), (190, 0, 160), 780))]
sc.camera = cams[0][1]
if DO_RENDER:
    for fname, cam in cams:
        sc.camera = cam; sc.render.filepath = os.path.join(PROJECT, fname); bpy.ops.render.render(write_still=True)
    sc.camera = cams[0][1]
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(PROJECT, "bike_generator_stand.blend"))
print("v5 build complete:", len(PARTS), "STL files;", "violations:", CHECKS["violations"])
