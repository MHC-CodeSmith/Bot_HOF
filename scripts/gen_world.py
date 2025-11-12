#!/usr/bin/env python3
import sys, math, yaml, pathlib

def box_model(name, size, pose, color=(0.7,0.7,0.7,1.0), static=True):
    sx, sy, sz = size
    x,y,z, roll,pitch,yaw = pose
    return f"""
  <model name='{name}'>
    <static>{str(static).lower()}</static>
    <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    <link name='{name}_link'>
      <visual name='vis'>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient></material>
      </visual>
      <collision name='col'>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
    </link>
  </model>
"""

def wall_models(L,W,H,th=0.08):
    halfL, halfW = L/2, W/2
    walls = []
    # parede +X
    walls.append(box_model("wall_pos_x",(th,W,H),( halfL, 0, H/2, 0,0,0),(0.8,0.8,0.85,1)))
    # -X
    walls.append(box_model("wall_neg_x",(th,W,H),(-halfL, 0, H/2, 0,0,0),(0.8,0.8,0.85,1)))
    # +Y
    walls.append(box_model("wall_pos_y",(L,th,H),(0, halfW, H/2, 0,0,0),(0.8,0.8,0.85,1)))
    # -Y
    walls.append(box_model("wall_neg_y",(L,th,H),(0,-halfW, H/2, 0,0,0),(0.8,0.8,0.85,1)))
    return "\n".join(walls)

def main():
    if len(sys.argv) != 3:
        print("uso: gen_world.py config/lab.yaml worlds/my_lab.sdf")
        sys.exit(1)

    cfg = yaml.safe_load(open(sys.argv[1]))
    out_path = pathlib.Path(sys.argv[2])

    L = float(cfg["room"]["length"])
    W = float(cfg["room"]["width"])
    H = float(cfg["room"]["height"])
    door = cfg["room"].get("door", None)

    bench = cfg["bench"]
    bL,bW,bH = float(bench["length"]), float(bench["width"]), float(bench["height"])
    bx = float(bench["pose"]["x"])
    by = float(bench["pose"]["y"])
    yaw = math.radians(float(bench["pose"].get("yaw_deg",0.0)))

    modules = cfg["modules"]
    mcount = int(modules["count"])
    mspacing = float(modules["spacing"])
    msx,msy,msz = float(modules["size"]["x"]), float(modules["size"]["y"]), float(modules["size"]["z"])

    eq = cfg["equipment"]

    world = [f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_lab">
    <gravity>0 0 -9.81</gravity>

    <light type="directional" name="sun">
      <pose>0 0 {H} 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.5 0.2 -1</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Piso -->
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="col"><geometry><plane><normal>0 0 1</normal><size>{L} {W}</size></plane></geometry></collision>
        <visual name="vis"><geometry><plane><normal>0 0 1</normal><size>{L} {W}</size></plane></geometry></visual>
      </link>
    </model>

    <!-- Paredes -->
{wall_models(L,W,H)}
"""]

    # Porta: abrir um "vão" (simplificado: colocamos um bloco transparente para referência)
    if door:
        dx,dy = float(door["x"]), float(door["y"])
        dwidth = float(door.get("width",0.9))
        dheight = float(door.get("height",2.1))
        world.append(box_model("door_frame",(0.06,dwidth,dheight),(dx,dy,dheight/2,0,0,0),(0.6,0.6,0.6,0.3)))

    # Bancada
    world.append(box_model("bench",(bL,bW,bH),(bx,by,bH/2,0,0,yaw),(0.2,0.4,0.8,1)))

    # Módulos sobre a bancada (alinhados ao eixo local da bancada)
    # offset inicial: centro da bancada menos metade do conjunto
    total_len = (mcount-1)*mspacing
    start = -total_len/2.0
    for i in range(mcount):
        ox = start + i*mspacing
        # gira offsets pelo yaw da bancada
        rx = ox*math.cos(yaw)
        ry = ox*math.sin(yaw)
        mx = bx + rx
        my = by + ry
        mz = bH + msz/2.0
        world.append(box_model(f"module_{i+1}",(msx,msy,msz),(mx,my,mz,0,0,yaw),(0.6,0.6,0.7,1)))

    # Equipamentos simplificados
    if eq.get("conveyor", False):
        cl = 1.2; cw = 0.3; ch = 0.2
        cx = bx; cy = by; cz = bH + ch/2.0
        world.append(box_model("conveyor",(cl,cw,ch),(cx,cy,cz,0,0,yaw),(0.1,0.6,0.1,1)))

    if eq.get("press", False):
        px = bx + 0.6*math.cos(yaw); py = by + 0.6*math.sin(yaw); pz = bH + 0.8/2
        world.append(box_model("press",(0.25,0.25,0.8),(px,py,pz,0,0,yaw),(0.6,0.1,0.1,1)))

    if eq.get("elevator", False):
        ex = bx - 0.6*math.cos(yaw); ey = by - 0.6*math.sin(yaw); ez = bH + 1.1/2
        world.append(box_model("elevator",(0.30,0.30,1.1),(ex,ey,ez,0,0,yaw),(0.4,0.4,0.4,1)))

    if eq.get("sensor", False):
        sx = bx; sy = by + 0.4; sz = bH + 0.1
        world.append(box_model("sensor",(0.08,0.08,0.2),(sx,sy,sz,0,0,0),(0.9,0.9,0.2,1)))

    if eq.get("arm", False):
        ax = bx + 0.1; ay = by - 0.4; az = bH + 0.6/2
        world.append(box_model("arm_placeholder",(0.25,0.25,0.6),(ax,ay,az,0,0,0),(0.2,0.2,0.2,1)))

    if eq.get("hmi", False):
        hx =  L/2 - 0.05; hy = 0.0; hz = 1.5
        world.append(box_model("hmi",(0.04,0.30,0.20),(hx,hy,hz,0,0,math.pi/2),(0.1,0.1,0.1,1)))

    # PLC (só um bloco na parede)
    plc_label = (cfg.get("equipment",{}).get("plc","PLC"))
    px =  L/2 - 0.05; py = 0.5; pz = 1.4
    world.append(box_model(f"plc_{plc_label}",(0.05,0.28,0.18),(px,py,pz,0,0,math.pi/2),(0.3,0.3,0.35,1)))

    world.append("""
  </world>
</sdf>
""")
    out_path.write_text("".join(world))
    print(f"[ok] mundo salvo em: {out_path}")

if __name__ == "__main__":
    main()

