-- we assume our robot to be rather cubical
function get_box_inertia(mass, width, height, depth)
  local valx, valy, valz
  valx = 1.0 / 12.0 * mass * (height * height + depth * depth)
  valy = 1.0 / 12.0 * mass * (width * width + height * height)
  valz = 1.0 / 12.0 * mass * (width * width + depth * depth)
  return {
    { valx, 0.0, 0.0 },
    { 0.0, valy, 0.0 },
    { 0.0, 0.0, valz }
  }
end

-- define model variables here for each segment
pelvis_w = 0.5
pelvis_d = 0.5
pelvis_h = 0.5
pelvis_m = 10.0

base_m = 5.0
joint_r = 0.2

upper_leg_w = 0.15
upper_leg_d = 0.15
upper_leg_h = 0.5
upper_leg_m = 10.0

lower_leg_w = upper_leg_w
lower_leg_d = upper_leg_d
lower_leg_h = upper_leg_h
lower_leg_m = upper_leg_m

foot_w = 0.4
foot_d = 0.2
foot_h = 0.1
foot_m = 1.0



-- fill in for thigh, shank, foot


-- add other bodies
pelvis    = { mass = pelvis_m, com = { 0.0, 0.0, pelvis_h / 2 }, inertia = get_box_inertia(pelvis_m, pelvis_w, pelvis_h, pelvis_d) }
base_link = { mass = base_m, com = { 0, 0, 0 }, inertia = get_box_inertia(pelvis_m, pelvis_w, pelvis_h, pelvis_d) }
thigh     = { mass = upper_leg_m, com = { 0, 0, -upper_leg_h / 2 }, inertia = get_box_inertia(upper_leg_m, upper_leg_w, upper_leg_h, upper_leg_d) }
shank     = { mass = upper_leg_m, com = { 0, 0, -lower_leg_h / 2 }, inertia = get_box_inertia(lower_leg_m, lower_leg_w, lower_leg_h, lower_leg_d) }
foot      = { mass = upper_leg_m, com = { 0, 0, -foot_h / 2 }, inertia = get_box_inertia(foot_m, foot_w, foot_h, foot_d) }


-- add more meshes
meshes = {
  Pelvis = {
    name = "Pelvis",
    dimensions = { pelvis_w, pelvis_d, pelvis_h },
    color = { 0.8, 0.8, 0.2 },
    mesh_center = { 0, 0, 0 },
    src = "meshes/unit_cube.obj",
  },
  Base = {
    name = "Base",
    dimensions = { pelvis_w, pelvis_d, pelvis_h / 4.0 },
    color = { 0.8, 0.8, 0.2 },
    mesh_center = { 0, 0, 0 },
    src = "meshes/unit_sphere_medres.obj",
  },
  Joint = {
    name = "Joint",
    dimensions = { joint_r, joint_r, joint_r },
    color = { 0, 1, 0 },
    mesh_center = { 0, 0, 0 },
    src = "meshes/unit_sphere_medres.obj",
  },
  UpperLeg = {
    name = "UpperLeg",
    dimensions = { upper_leg_w, upper_leg_d, upper_leg_h },
    color = { 0, 0, 1 },
    mesh_center = { 0, 0, -upper_leg_h / 2 },
    src = "meshes/unit_cube.obj",
  },
  LowerLeg = {
    name = "LowerLeg",
    dimensions = { lower_leg_w, lower_leg_d, lower_leg_h },
    color = { 0, 0, 1 },
    mesh_center = { 0, 0, -lower_leg_h / 2 },
    src = "meshes/unit_cube.obj",
  },
  Foot = {
    name = "Foot",
    dimensions = { foot_w, foot_d, foot_h },
    color = { 1, 0, 0 },
    mesh_center = { 0, -foot_d / 2, -foot_h / 2 },
    src = "meshes/unit_cube.obj",
  },
}

-- these are the bodies your robot should have
bodies = {
  base_link = base_link,
  pelvis = pelvis,
  thigh_right = thigh,
  shank_right = shank,
  foot_right = foot,
  thigh_left = thigh,
  shank_left = shank,
  foot_left = foot
}

joints = {
  floating_base = {
    { 0., 0., 0., 1., 0., 0. },
    { 0., 0., 0., 0., 1., 0. },
    { 0., 0., 0., 0., 0., 1. },
    { 0., 0., 1., 0., 0., 0. },
    { 0., 1., 0., 0., 0., 0. },
    { 1., 0., 0., 0., 0., 0. }
  },
  -- hip joint and pelvis joint
  spherical_xyz = {
    { 1., 0., 0., 0., 0., 0. },
    { 0., 1., 0., 0., 0., 0. },
    { 0., 0., 1., 0., 0., 0. }
  },
  -- ankle joint
  rotational_xy = {
    { 1., 0., 0., 0., 0., 0. },
    { 0., 1., 0., 0., 0., 0. }
  },
  -- knee joint
  rotational_y = {
    { 0., 1., 0., 0., 0., 0. }
  },
  fixed = {}
}

model = {
  configuration = {
    axis_front     = { 1, 0, 0 },
    axis_up        = { 0, 0, 1 },
    axis_right     = { 0, 1, 0 },
    rotation_order = { 2, 1, 0 },
  },
  frames = {
    {
      -- starting link of your robot
      name = "base_link",
      parent = "ROOT",
      body = bodies.base_link,
      joint_frame = {
        r = { 0.0, 0.0, upper_leg_h + lower_leg_h + foot_h },
      },
      joint = joints.floating_base,
      visuals = { meshes.Base }
    },
    {
      -- starting link of your robot
      name = "pelvis",
      parent = "base_link",
      body = bodies.pelvis,
      joint_frame = {
        r = { 0.0, 0.0, pelvis_h / 2 },
      },
      joint = joints.spherical_xyz,
      visuals = { meshes.Pelvis }
    },
    {
      -- another link
      name = "thigh_right",
      parent = "base_link",
      body = bodies.thigh_right,
      joint = joints.spherical_xyz,
      joint_frame = {
        r = { 0.0, -pelvis_w / 2.0, 0 },
      },
      visuals = { meshes.UpperLeg, meshes.Joint }
    },
    {
      -- another link
      name = "shank_right",
      parent = "thigh_right",
      body = bodies.shank_right,
      joint = joints.rotational_y,
      joint_frame = {
        r = { 0.0, 0, -upper_leg_h },
      },
      visuals = { meshes.LowerLeg, meshes.Joint }
    },

    {
      name = "ankle_right",
      parent = "shank_right",
      joint = joints.rotational_xy,
      joint_frame = {
        r = { 0, 0, -lower_leg_h },
      },
      visuals = { meshes.Joint }
    },
    {
      name = "foot_right",
      parent = "ankle_right",
      body = bodies.foot_right,
      joint = joints.fixed,
      joint_frame = {
        r = { lower_leg_d / 2, foot_d / 2, 0 }
      },
      visuals = { meshes.Foot }
    },
    {
      name = "thigh_left",
      parent = "base_link",
      body = bodies.thigh_left,
      joint = joints.spherical_xyz,
      joint_frame = {
        r = { 0.0, pelvis_w / 2.0, 0 },
      },
      visuals = { meshes.UpperLeg, meshes.Joint }
    },
    {
      name = "shank_left",
      parent = "thigh_left",
      body = bodies.shank_left,
      joint = joints.rotational_y,
      joint_frame = {
        r = { 0.0, 0, -upper_leg_h },
      },
      visuals = { meshes.LowerLeg, meshes.Joint }
    },
    {
      name = "ankle_left",
      parent = "shank_left",
      joint = joints.rotational_xy,
      joint_frame = {
        r = { 0, 0, -lower_leg_h },
      },
      visuals = { meshes.Joint }
    },
    {
      name = "foot_left",
      parent = "ankle_left",
      body = bodies.foot_left,
      joint = joints.fixed,
      joint_frame = {
        r = { lower_leg_d / 2, foot_d / 2, 0 }
      },
      visuals = { meshes.Foot }
    },
  }
}

return model
