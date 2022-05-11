
-- we assume our robot to be rather cubical
function get_box_inertia (mass, width, height, depth)
    local valx, valy, valz
    valx = 1.0/12.0 * mass * (height*height + depth*depth)
    valy = 1.0/12.0 * mass * (width*width + height*height)
    valz = 1.0/12.0 * mass * (width*width + depth*depth)
    return {
        {valx,  0.0,  0.0},
        { 0.0, valy,  0.0},
        { 0.0,  0.0, valz}
    }
end

-- define model variables here for each segment
pelvis_w = 0.5
pelvis_d = 0.5
pelvis_h = 0.5
pelvis_m = 10.0

base_m = 5.0
joint_r = 0.2

-- fill in for thigh, shank, foot


-- add other bodies
pelvis    = { mass = pelvis_m, com = ___,   inertia = ___ }
base_link = { mass = base_m,   com = ___,   inertia = ___ }
 

-- add more meshes
meshes = {
  Pelvis = {
    name = "Pelvis",
    dimensions = { pelvis_w, pelvis_d, pelvis_h },
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, ___ },
    src = "meshes/unit_cube.obj",
  },  
  Base = {
    name = "Base",
    dimensions = { pelvis_w, pelvis_d, pelvis_h/4.0 },
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, ___ },
    src = "meshes/unit_sphere_medres.obj",
  },
  Joint = {
    name = "Joint",
    dimensions = {joint_r, joint_r, joint_r},
    color = { 0, 1, 0},
    mesh_center = { 0, 0, 0 },
    src = "meshes/unit_sphere_medres.obj",
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
		{ 0., 0., 0., 1., 0., 0.},
		{ 0., 0., 0., 0., 1., 0.},
		{ 0., 0., 0., 0., 0., 1.},
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.}
	},
    -- hip joint and pelvis joint
	spherical_xyz = {
	},
    -- ankle joint
	rotational_xy = {
	},
    -- knee joint
	rotational_y = {
	},
	fixed = {}
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
            -- starting link of your robot
			name = "base_link",
			parent = "ROOT",
			body = bodies.base_link,
            joint_frame = {
              r = { 0.0, 0.0 , 0.0 },
            },
			joint = joints.floating_base,
            visuals = { meshes.Base}
		},
		{
            -- another link
			name = "thigh_right",
			parent = "base_link",
			body = bodies.thigh_right,
			joint = joints.spherical_xyz,
            joint_frame = {
              r = { 0.0, -pelvis_w/2.0, 0 },
            },
            visuals = { meshes.UpperLeg, meshes.Joint}
		},
        -- add the kinematic tree to your robot
	}
}

return model
