# box10: giant cardboard box, 10 clips, one policy (formhoi/train_dex_object.sh)

Motion files: src/holosoma/holosoma/data/motions/g1_inspire/formhoi_box10/ (dexterous OmniRetarget, G1 + Inspire, 50 fps)
Object: src/holosoma/holosoma/data/scene_objects/formhoi/giant_cardboard_box/giant_cardboard_box.urdf
  - real size (OmniRetarget keeps the object unscaled), the same scan our mjlab hoi_box10 uses for all 10 clips
    (FORM-HOI has two scans of this box: push_04, rotate_midair_08 and up_down_09 were captured with one ~1 cm/side wider)
  - mass 1.0 kg (as our mjlab runs; the retargeter's URDF template has 0.1 kg), box inertia from its extents

Clips (retargeter settings default unless noted):
  fd44 pick_place_ground_desk_10   (ends on a desk; the scene has no desk)
  fd45 pick_place_desk_ground_05   (starts on a desk; the scene has no desk)
  fd47 pull_03
  fd50 hold_over_head_02
  fd59 back_lift_01                penetration_tolerance 5 mm, foot_sticking_tolerance 2 cm (infeasible at defaults)
  fd60 push_04
  fd61 rotate_ground_07            penetration_tolerance 5 mm, foot_sticking_tolerance 2 cm (infeasible at defaults)
  fd62 rotate_midair_08
  fd63 up_down_09
  fd64 vertical_horizontal_02
