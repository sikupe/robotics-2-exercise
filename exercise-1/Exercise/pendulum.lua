meshes = {
  Base = {
    name = "Base",
    dimensions = { 0.05, 0.05, 0.05 },
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, 0 },
    src = "meshes/unit_cube.obj",
  },
  UpperArm = {
    name = "UpperArm",
    dimensions = { 0.01, 0.01, 0.4 },
    color = { 0.1, 0.1, 0.8},
    mesh_center = { 0, 0, -0.2 },
    src = "meshes/unit_cube.obj",
  },
  LowerArm = {
    name = "LowerArm",
    dimensions = { 0.01, 0.01, 0.3 },
    color = { 0.2, 0.2, 0.9},
    mesh_center = { 0, 0, -0.15 },
    src = "meshes/unit_cube.obj",
  },
  Joint_Start = {
    name = "Joint_Start",
    dimensions = {0.05, 0.05, 0.05},
    color = { 0, 1, 0},
    mesh_center = { 0, 0, 0 },
    src = "meshes/unit_sphere_medres.obj",
  },
  Joint_End_LowerArm = {
    name = "Joint_End_LowerArm",
    dimensions = {0.05, 0.05, 0.05},
    color = { 0, 1, 0},
    mesh_center = { 0, 0, -0.3 },
    src = "meshes/unit_sphere_medres.obj",
  },
}

model = {
  configuration = {
    axis_front = { 1, 0, 0 },
    axis_up    = { 0, 0, 1 },
    axis_right = { 0, 1, 0 },
    rotation_order = { 2, 1, 0},
  },

  frames = {
    {
      name = "BASE",
      parent = "ROOT",
      joint_frame = {
        r = { 0, 0 , 1 },
      },
      visuals = {
        meshes.Base,
      },
    },
    {
      name = "UPPERARM",
      parent = "BASE",
      joint_frame = {
        r = { 0, 0 , 0 },
      },
      joint = {
        { 1, 0, 0, 0, 0, 0,},
      },
      visuals = {
        meshes.UpperArm,
        meshes.Joint_Start
      },
    },
    {
      name = "LOWERARM",
      parent = "UPPERARM",
      joint_frame = {
        r = { 0, 0 , -0.4 },
      },
      joint = {
        { 1, 0, 0, 0, 0, 0,},
      },
      visuals = {
        meshes.LowerArm,
        meshes.Joint_Start,
        meshes.Joint_End_LowerArm
      },
    },
  }
}

return model
