Body_l = 0.8
Body_w = 0.12
Body_m = 8

Leg_l = 0.6
Leg_w = 0.08
Leg_m = 3

function cylinder_inertia (mass, width, height)
	return {
		{ 1. / 12. * mass * ( 3. * width * width / 4. + height * height), 0., 0.},
		{ 0., 1. / 12. * mass * (3. * width * width / 4. + height * height) , 0.},
		{ 0., 0., 1. / 2. * mass * (width * width / 4.)}
	}
end

Leg = { 
	mass = Leg_m, 
	com = { 0., 0., -Leg_l * 0.5}, 
	inertia = cylinder_inertia (Leg_m, Leg_w, Leg_l)
}
Body = {
	mass = Body_m,
	com = { 0., 0., Body_l * 0.5},
	inertia = cylinder_inertia (Body_m, Body_w, Body_l)
}

bodies = {
	Leg = Leg,
	Body = Body,
}


joints = {
	trans_z = {
	    { 0., 0., 0., 0., 0., 1.}
	},
}

meshes = {
	Leg = {
		name = "Leg",
		dimensions = { Leg_w, Leg_w, Leg_l},
		color = { 0.8, 0.8, 0.2},
		mesh_center = { 0., 0., -Leg_l * 0.5},
		rotate = { axis = {1., 0., 0.}, angle = 90.},
		src = "cylinder.obj"		
	},
	Body = {
		name = "Body",
		dimensions = { Body_w, Body_w, Body_l},
		color = { 0.8, 0.2, 0.8},
		mesh_center = { 0., 0., Body_l * 0.5},
		rotate = { axis = {1., 0., 0.}, angle = 90.},
		src = "cylinder.obj"
	},
}

model = {
	configuration = {
		axis_front = { 1, 0, 0},
		axis_up = { 0, 0, 1},
		axis_right = { 0, 1, 0},
	},
	gravity = {0., 0., -9.81},
	frames = {
		{
			name = "Body",
			parent = "ROOT",
			body = Body,
			joint = joints.trans_z,
			visuals = { meshes.Body },
		},
		{
			name = "Leg",
			parent = "Body",
			body = Leg,
			joint = joints.trans_z,
			visuals = { meshes.Leg },
			joint_frame = {
				r = { 0., 0., 0 * Leg_l},
			},
		},
		{
			name = "LegEnd",
			parent = "Leg",
			joint_frame = {
				r = { 0., 0., -Leg_l},
			},
		},
		{
			name = "BodyEnd",
			parent = "Body",
			joint_frame = {
				r = { 0., 0., Body_l},
			},
		},
	},
}

return model
