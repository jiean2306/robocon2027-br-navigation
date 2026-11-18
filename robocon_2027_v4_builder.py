#!/usr/bin/env python3
"""
ABU Robocon 2027 V4 arena correction for Gazebo Harmonic / SDF 1.9.

Builds from the user's original field_v2 SDF (not the flawed V3 additions),
preserving its existing upper-level structures while correcting materials and
adding the missing ground-level field details, blocks, and Mustika.

The script does not overwrite its input. It writes a new V4 SDF and a JSON
validation report. Coordinates are metres; RGB values come from Rulebook V1.1
Section 14. Layout dimensions/relationships come from Appendix V1.1 Figures
3.1, 3.2, 3.9 and 3.10. Where the Appendix does not specify object stocking
order or fence details, assumptions are explicitly listed in the report.
"""
from __future__ import annotations
import argparse, json, math, shutil
import xml.etree.ElementTree as ET
from pathlib import Path

RGB = {
    "ground_red": (240,210,210), "ground_blue": (170,210,230),
    "shared": (245,240,200), "retry_red": (223,34,34),
    "retry_blue": (50,0,255), "l1_red": (235,180,160),
    "l1_blue": (150,215,220), "green": (40,100,50),
    "transfer_red": (245,170,60), "transfer_blue": (60,170,245),
    "ramp_red": (200,150,140), "ramp_blue": (130,180,200),
    "grey": (190,190,185), "brown": (100,62,0),
    "mustika": (245,205,0), # visual approximation only; Rulebook says original Mikasa ball colour
    "earth_red": (220,35,35), "earth_blue": (35,45,220),
    "sky_red": (220,35,35), "sky_blue": (35,45,220),
}
def rgba(key):
    r,g,b=RGB[key]
    return (r/255,g/255,b/255,1.0)
def fmt(v): return f"{v:.6f}".rstrip("0").rstrip(".") if isinstance(v,float) else str(v)
def pose(x,y,z,roll=0,pitch=0,yaw=0):
    return " ".join(fmt(float(v)) for v in (x,y,z,roll,pitch,yaw))
def material(visual,key):
    old=visual.find("material")
    if old is not None: visual.remove(old)
    m=ET.SubElement(visual,"material")
    c=" ".join(f"{v:.6f}" for v in rgba(key))
    ET.SubElement(m,"ambient").text=c
    ET.SubElement(m,"diffuse").text=c
    ET.SubElement(m,"specular").text="0.05 0.05 0.05 1"
    ET.SubElement(m,"emissive").text="0 0 0 1"
def geom_box(parent,sx,sy,sz):
    g=ET.SubElement(parent,"geometry"); b=ET.SubElement(g,"box")
    ET.SubElement(b,"size").text=f"{sx:.6f} {sy:.6f} {sz:.6f}"
def add_link(model,name,x,y,z,sx,sy,sz,color,collision=True,yaw=0):
    link=ET.SubElement(model,"link",{"name":name})
    ET.SubElement(link,"pose").text=pose(x,y,z,0,0,yaw)
    vis=ET.SubElement(link,"visual",{"name":"visual"}); geom_box(vis,sx,sy,sz); material(vis,color)
    if collision:
        col=ET.SubElement(link,"collision",{"name":"collision"}); geom_box(col,sx,sy,sz)
    return link
def add_cylinder(model,name,x,y,z,radius,length,color):
    link=ET.SubElement(model,"link",{"name":name})
    ET.SubElement(link,"pose").text=pose(x,y,z)
    for tag in ("visual","collision"):
        e=ET.SubElement(link,tag,{"name":tag})
        g=ET.SubElement(e,"geometry"); c=ET.SubElement(g,"cylinder")
        ET.SubElement(c,"radius").text=fmt(radius); ET.SubElement(c,"length").text=fmt(length)
        if tag=="visual": material(e,color)
def add_sphere(world,name,x,y,z,radius,mass,color):
    m=ET.SubElement(world,"model",{"name":name})
    ET.SubElement(m,"pose").text=pose(x,y,z)
    link=ET.SubElement(m,"link",{"name":"link"})
    for tag in ("visual","collision"):
        e=ET.SubElement(link,tag,{"name":tag})
        g=ET.SubElement(e,"geometry"); s=ET.SubElement(g,"sphere")
        ET.SubElement(s,"radius").text=fmt(radius)
        if tag=="visual": material(e,color)
    ET.SubElement(link,"gravity").text="true"
    ET.SubElement(link,"inertial")
    inert=link.find("inertial")
    ET.SubElement(inert,"mass").text=fmt(mass)
    ii=ET.SubElement(inert,"inertia"); I=0.4*mass*radius*radius
    for k,v in (("ixx",I),("iyy",I),("izz",I),("ixy",0),("ixz",0),("iyz",0)):
        ET.SubElement(ii,k).text=fmt(v)
    return m
def get_world(root):
    if root.tag=="world": return root
    w=root.find("world")
    if w is None: raise ValueError("Input SDF has no <world> element")
    return w
def clean_previous_generated(world):
    # Only remove known generator-created entities from old V3; start from V2 by default.
    prefixes=("earth_red_","earth_blue_","sky_block_","arena_completion_static_additions",
              "mustika_mikasa_v300w","v4_")
    for m in list(world.findall("model")):
        if (m.get("name") or "").startswith(prefixes):
            world.remove(m)
def recolor_existing(world):
    # Recolor original V2 structures according to Rulebook Section 14.
    for model in world.findall("model"):
        n=(model.get("name") or "").lower()
        key=None
        if n=="ground_field": key="grey"
        elif n=="level_1": key="l1_red" # floor is subsequently split by non-collision overlays
        elif n=="level_2": key="grey"
        elif "barrier" in n or "side_wall" in n: key="grey"
        elif "stair" in n:
            key="l1_red" if n.startswith("red") else "l1_blue"
        elif "ramp" in n:
            key="ramp_red" if n.startswith("red") else "ramp_blue"
        elif "transfer_area" in n:
            key="transfer_red" if n.startswith("red") else "transfer_blue"
        elif "retry_zone" in n:
            key="retry_red" if "red" in n else "retry_blue"
        elif "building_spot" in n: key="green"
        elif "central_pillar" in n: key="brown"
        if key:
            for v in model.findall(".//visual"):
                material(v,key)

def add_static_arena(world):
    # Ground coordinates follow Appendix plan: x=-5.5..+5.5; y=-5.5..+5.5.
    # Red is x<0, Blue x>0. Ground slab top is z=0.
    arena=ET.SubElement(world,"model",{"name":"v4_ground_details"})
    ET.SubElement(arena,"static").text="true"
    # Floor colour regions; thin visual-only overlays do not add collision surfaces.
    add_link(arena,"ground_red_surface",-2.75,0,0.001,5.45,10.9,0.002,"ground_red",False)
    add_link(arena,"ground_blue_surface",2.75,0,0.001,5.45,10.9,0.002,"ground_blue",False)
    # Ground Shared Area: 1200 x 1200, 650 mm from the bottom inner boundary.
    gx,gy=0.0,-4.25
    add_link(arena,"ground_shared_surface",gx,gy,0.003,1.2,1.2,0.003,"shared",False)
    # 5x5 individual 200 mm cells. The 12 Sky Block cells are light yellow;
    # other cells are grey. The centre cell is intentionally left unoccupied.
    sky_cells={(0,0),(0,2),(0,4),(1,1),(1,3),(2,0),(2,4),
               (3,1),(3,3),(4,0),(4,2),(4,4)}
    for row in range(5):
        for col in range(5):
            x=gx-0.5+(col+0.5)*0.2
            y=gy+0.5-(row+0.5)*0.2
            cell_color="shared" if (row,col) in sky_cells else "grey"
            if (row,col)==(2,2): cell_color="brown"
            add_link(arena,f"sky_grid_cell_{row+1}_{col+1}",x,y,0.005,0.198,0.198,0.002,cell_color,False)
    # Fine grid lines make the 200 mm pitch readable without collision surfaces.
    for i in range(6):
        x=gx-0.6+i*0.2; y=gy-0.6+i*0.2
        add_link(arena,f"sky_grid_vertical_{i+1}",x,gy,0.007,0.003,1.2,0.002,"grey",False)
        add_link(arena,f"sky_grid_horizontal_{i+1}",gx,y,0.007,1.2,0.003,0.002,"grey",False)
    # Mustika Ground Shared Area: 1000 x 1000, 750 mm from top inner boundary.
    mx,my=0.0,4.25
    add_link(arena,"mustika_ground_shared_surface",mx,my,0.003,1.0,1.0,0.003,"shared",False)
    add_cylinder(arena,"mustika_pillar",mx,my,0.25,0.135,0.50,"brown")
    # Receptacle approximation: 12 short rim segments; central opening Ø180 mm.
    mid=(0.135+0.09)/2; radial=0.135-0.09; n=12
    seglen=2*mid*math.sin(math.pi/n)*1.03
    for i in range(n):
        a=2*math.pi*i/n
        add_link(arena,f"mustika_receptacle_rim_{i+1}",
                 mx+mid*math.cos(a),my+mid*math.sin(a),0.45,
                 radial,seglen,0.10,"brown",True,a)
    # Start/Retry zones: 2 x 700 square, 100 mm gap, placed against bottom boundary.
    # Appendix 3.2(a) shows the Blue arrangement; Red is its mirror.
    for team,centres,key in (
        ("red",[-5.10,-4.30],"retry_red"),
        ("blue",[4.30,5.10],"retry_blue")):
        for i,x in enumerate(centres,1):
            add_link(arena,f"{team}_ground_start_retry_{i}",x,-5.10,0.004,
                     0.70,0.70,0.008,key,False)
    # Storage Areas: 2000 mm along x by 1000 mm along y, against the top and outer side boundaries.
    add_link(arena,"red_storage_surface",-4.45,4.95,0.004,2.0,1.0,0.008,"ground_red",False)
    add_link(arena,"blue_storage_surface",4.45,4.95,0.004,2.0,1.0,0.008,"ground_blue",False)
    # Ground center divider fence, 30 mm thick and 50 mm high. Appendix says gaps at
    # Ground Shared Area and Mustika pillar area. The segment endpoints follow those gaps.
    # This is a geometric interpretation of Figure 3.9; report flags it for visual review.
    divider_segments=[(-5.45,-4.85),(-3.65,3.65),(4.75,5.45)]
    for i,(a,b) in enumerate(divider_segments,1):
        add_link(arena,f"ground_center_divider_{i}",0,(a+b)/2,0.025,
                 0.03,b-a,0.05,"brown",True)
    # Outer Game Field Boundary: 50 mm thick, 100 mm high per Fig 3.10(a).
    # Centreline ends are joined at the corners; inner playing dimensions remain 11 m.
    h,t=0.10,0.05
    add_link(arena,"outer_boundary_north",0,5.475,h/2,11.0,t,h,"brown",True)
    add_link(arena,"outer_boundary_south",0,-5.475,h/2,11.0,t,h,"brown",True)
    add_link(arena,"outer_boundary_west",-5.475,0,h/2,t,11.0,h,"brown",True)
    add_link(arena,"outer_boundary_east",5.475,0,h/2,t,11.0,h,"brown",True)
    return {"ground_shared_center_m":[gx,gy],"mustika_area_center_m":[mx,my],
            "storage_area_centres_m":{"red":[-4.45,4.95],"blue":[4.45,4.95]},
            "start_retry_centres_m":{"red":[[-5.10,-5.10],[-4.30,-5.10]],
                                     "blue":[[4.30,-5.10],[5.10,-5.10]]},
            "divider_segments_y_m":divider_segments}
def add_l1_l2_overlays(world):
    m=ET.SubElement(world,"model",{"name":"v4_upper_surface_overlays"})
    ET.SubElement(m,"static").text="true"
    # Existing L1 top is z=0.6. Overlay team-coloured floor halves.
    add_link(m,"l1_red_surface",-1.485,0,0.601,2.97,5.94,0.002,"l1_red",False)
    add_link(m,"l1_blue_surface",1.485,0,0.601,2.97,5.94,0.002,"l1_blue",False)
    # Existing V2 building spots, retry zones, shared strips and transfer pads
    # are retained and recoloured in-place by recolor_existing(); do not duplicate them.
    # L2 platform top is z=0.90; four 500x500 building spots, green.
    for i,(x,y) in enumerate([(-1.0,-1.0),(-1.0,1.0),(1.0,-1.0),(1.0,1.0)],1):
        add_link(m,f"l2_building_spot_surface_{i}",x,y,0.906,0.5,0.5,0.012,"green",False)
    return m
def add_sky_blocks(world):
    # 12 positions are the light-yellow cells shown in Appendix Figure 3.2(c).
    # Grid row 0 is top (+y), row 4 bottom (-y); the centre cell is empty.
    cells=[(0,0),(0,2),(0,4),(1,1),(1,3),(2,0),(2,4),
           (3,1),(3,3),(4,0),(4,2),(4,4)]
    gx,gy=0.0,-4.25
    for n,(row,col) in enumerate(cells,1):
        x=gx-0.5+(col+0.5)*0.2
        y=gy+0.5-(row+0.5)*0.2
        m=ET.SubElement(world,"model",{"name":f"sky_block_{n:02d}"})
        ET.SubElement(m,"pose").text=pose(x,y,0.10)
        link=ET.SubElement(m,"link",{"name":"link"})
        # One full-cube collision and neutral cube body, with red/blue opposite
        # side-face panels. This models "red on one side and blue on the other"
        # rather than stacking coloured halves and making a purple-looking block.
        col=ET.SubElement(link,"collision",{"name":"collision"}); geom_box(col,0.2,0.2,0.2)
        body=ET.SubElement(link,"visual",{"name":"body"})
        geom_box(body,0.2,0.2,0.2); material(body,"grey")
        for label,x,key in (("red_side",-0.099,"sky_red"),("blue_side",0.099,"sky_blue")):
            vis=ET.SubElement(link,"visual",{"name":label})
            ET.SubElement(vis,"pose").text=pose(x,0,0)
            geom_box(vis,0.002,0.2,0.2); material(vis,key)
        ET.SubElement(link,"inertial")
        inert=link.find("inertial"); ET.SubElement(inert,"mass").text="0.22"
        ii=ET.SubElement(inert,"inertia"); I=0.22*0.2*0.2/6
        for k,v in (("ixx",I),("iyy",I),("izz",I),("ixy",0),("ixz",0),("iyz",0)):
            ET.SubElement(ii,k).text=fmt(v)
def add_earth_blocks(world):
    # Appendix specifies 20 blocks per team, 350 mm cubes. The rulebook does not
    # prescribe a precise packing/stacking pattern within each storage pad.
    # Use two layers of 10 (2 columns x 5 rows) inside each 2.0 x 1.0 m pad.
    side=0.35; pitch=0.375
    for team,cx,key in (("red",-4.45,"earth_red"),("blue",4.45,"earth_blue")):
        for idx in range(20):
            layer=idx//10; local=idx%10
            row=local//5; col=local%5
            x=cx+(col-2)*pitch
            y=4.95+(row-0.5)*pitch
            z=side/2+layer*side
            name=f"earth_{team}_{idx+1:02d}"
            model=ET.SubElement(world,"model",{"name":name})
            ET.SubElement(model,"pose").text=pose(x,y,z)
            link=ET.SubElement(model,"link",{"name":"link"})
            for tag in ("visual","collision"):
                e=ET.SubElement(link,tag,{"name":tag}); geom_box(e,side,side,side)
                if tag=="visual": material(e,key)
            ET.SubElement(link,"inertial")
            inert=link.find("inertial"); ET.SubElement(inert,"mass").text="0.60"
            ii=ET.SubElement(inert,"inertia"); I=0.60*side*side/6
            for k,v in (("ixx",I),("iyy",I),("izz",I),("ixy",0),("ixz",0),("iyz",0)):
                ET.SubElement(ii,k).text=fmt(v)
def add_mustika(world):
    # Ball shown resting in the Ground Mustika Pillar receptacle.
    add_sphere(world,"mustika_mikasa_v300w",0,4.25,0.605,0.105,0.27,"mustika")
def validate(root,report):
    world=get_world(root)
    models=world.findall("model"); names=[m.get("name") for m in models]
    errors=[]; warnings=[]
    if len(names)!=len(set(names)): errors.append("Duplicate top-level model names")
    earth=[n for n in names if n and n.startswith("earth_")]
    sky=[n for n in names if n and n.startswith("sky_block_")]
    if len(earth)!=40: errors.append(f"Expected 40 Earth Blocks, found {len(earth)}")
    if len(sky)!=12: errors.append(f"Expected 12 Sky Blocks, found {len(sky)}")
    if names.count("mustika_mikasa_v300w")!=1: errors.append("Expected exactly one Mustika ball")
    # Verify every dynamic block is within the 11m field footprint.
    for m in models:
        n=m.get("name","")
        if n.startswith(("earth_","sky_block_")):
            p=[float(x) for x in m.findtext("pose","0 0 0").split()[:3]]
            half=0.175 if n.startswith("earth_") else 0.1
            if abs(p[0])+half>5.5 or abs(p[1])+half>5.5:
                errors.append(f"{n} is outside the 11m field: x={p[0]:.3f}, y={p[1]:.3f}")
    # Ensure sky block visual halves are equal and non-overlapping vertically.
    for m in models:
        if m.get("name","").startswith("sky_block_"):
            link=m.find("link")
            visuals=link.findall("visual") if link is not None else []
            if len(visuals)!=3: errors.append(f"{m.get('name')} should have a body plus two opposite colour faces")
    # Required named static features.
    static_model=world.find("model[@name='v4_ground_details']")
    required=["ground_red_surface","ground_blue_surface","ground_shared_surface",
              "mustika_ground_shared_surface","red_storage_surface","blue_storage_surface",
              "outer_boundary_north","outer_boundary_south","outer_boundary_west",
              "outer_boundary_east","ground_center_divider_1","ground_center_divider_2",
              "ground_center_divider_3"]
    found={l.get("name") for l in static_model.findall("link")} if static_model is not None else set()
    for item in required:
        if item not in found: errors.append(f"Missing required arena feature {item}")
    # Report known approximations / unresolved drawing interpretations.
    warnings += [
      "Earth Block initial stacking order is not prescribed by the Appendix; V4 uses two layers of 10 inside each storage pad. Confirm with the event's initial setup if specified separately.",
      "Mustika receptacle is approximated with 12 rim segments; verify exact concave cap profile against physical build.",
      "The center-divider segment lengths are a geometric interpretation of Appendix Figure 3.9; verify against the full dimension chain before treating as fabrication dimensions.",
      "Mustika ball appearance is a yellow visual approximation; Rulebook requires an original-condition Mikasa V300W and gives no RGB code.",
      "Ramp/stair geometry is retained from V2; its slope and transition geometry should be checked against Appendix Figures 3.7–3.8.",
      "L1/L2 spot positions and transfer-area positions are retained from V2 or reconstructed from the existing coordinate layout; visually verify against Appendix Figure 3.1."
    ]
    report.update({"model_count":len(models),"earth_blocks":len(earth),"sky_blocks":len(sky),
                   "mustika_ball_count":names.count("mustika_mikasa_v300w"),
                   "errors":errors,"warnings":warnings,
                   "checks":{"xml_parse":"PASS","unique_top_level_model_names":"PASS" if len(names)==len(set(names)) else "FAIL",
                             "earth_block_count":"PASS" if len(earth)==40 else "FAIL",
                             "sky_block_count":"PASS" if len(sky)==12 else "FAIL",
                             "all_blocks_inside_11m_field":"PASS" if not any("outside the 11m" in e for e in errors) else "FAIL",
                             "required_ground_features":"PASS" if not any("Missing required" in e for e in errors) else "FAIL"}})
    return report
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--input",required=True,help="Original V2 SDF path")
    ap.add_argument("--output",required=True,help="New V4 SDF path")
    ap.add_argument("--report",default=None,help="JSON validation report path")
    args=ap.parse_args()
    src=Path(args.input).expanduser().resolve(); out=Path(args.output).expanduser().resolve()
    if not src.is_file(): raise SystemExit(f"Input SDF not found: {src}")
    if src==out: raise SystemExit("Refusing to overwrite input SDF")
    tree=ET.parse(src); root=tree.getroot(); world=get_world(root)
    clean_previous_generated(world)
    recolor_existing(world)
    layout=add_static_arena(world)
    add_l1_l2_overlays(world)
    add_sky_blocks(world); add_earth_blocks(world); add_mustika(world)
    ET.indent(tree,space="  ")
    out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists():
        backup=out.with_suffix(out.suffix+".bak")
        shutil.copy2(out,backup)
        print(f"Existing output backed up to {backup}")
    tree.write(out,encoding="utf-8",xml_declaration=True)
    # Re-open the actual output, not only the in-memory tree.
    parsed=ET.parse(out); report=validate(parsed.getroot(),{"input":str(src),"output":str(out),"layout":layout})
    rp=Path(args.report).expanduser().resolve() if args.report else out.with_suffix(".validation.json")
    rp.write_text(json.dumps(report,indent=2))
    print("\n=== ABU ROBOCON 2027 V4 RESULT ===")
    print("Output:",out)
    print("Validation report:",rp)
    print("Models:",report["model_count"])
    print("Earth Blocks:",report["earth_blocks"],"/40")
    print("Sky Blocks:",report["sky_blocks"],"/12")
    print("Mustika balls:",report["mustika_ball_count"])
    print("Errors:",len(report["errors"]))
    print("Warnings:",len(report["warnings"]))
    for e in report["errors"]: print("ERROR:",e)
    for w in report["warnings"]: print("REVIEW:",w)
    if report["errors"]: raise SystemExit(2)
    print("XML and automated structural checks: PASS")
if __name__=="__main__": main()
