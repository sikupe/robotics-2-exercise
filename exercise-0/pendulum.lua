upper_length = 0.4
lower_length = 0.3
mesh_src = "unit_cube.obj"

meshes = {
    UpperPendulum = {
        name = "UpperPendulum",
        dimensions = { 0.1, 0.1, upper_length },
        color = { 1, 0, 0 },
        mesh_center = { 0, 0, -0.5 * upper_length },
        src = mesh_src
    },
    LowerPendulum = {
        name = "LowerPendulum",
        dimensions = { 0.1, 0.1, lower_length },
        color = { 0, 1, 0 },
        mesh_center = { 0, 0, -0.5 * lower_length },
        src = mesh_src
    }
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
            name = "UpperPendulum",
            parent = "ROOT",
            joint = {{ 1, 0, 0, 0, 0, 0 }},
            joint_frame = {
                r = { 0, 0, 1 },
                E = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
            },
            visuals = {
                meshes.UpperPendulum
            }
        },
        {
            name = "LowerPendulum",
            parent = "UpperPendulum",
            joint = {{ 1, 0, 0, 0, 0, 0 }},
            joint_frame = {
                r = { 0, 0, -upper_length },
                E = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
            },
            visuals = {
                meshes.LowerPendulum
            }
        }
    }
}

return model