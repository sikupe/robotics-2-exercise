require 'SRC.strict'

-- define some convenience functions

function get_sphere_inertia (mass, radius)
    local val
    val = ____
    return {
        {val, 0.0, 0.0},
        {0.0, val, 0.0},
        {0.0, 0.0, val}
    }
end

function get_box_inertia (mass, width, height, depth)
    local valx, valy, valz
    valx = ___
    valy = ___
    valz = ___
    return {
        {valx,  0.0,  0.0},
        { 0.0, valy,  0.0},
        { 0.0,  0.0, valz}
    }
end

-- define some constants

cart_w = ____ -- x-axis
cart_d = ____ -- y-axis
cart_h = ____ -- z-axis
cart_m = ____
cart_com = {
    0.0, 0.0, -cart_h/4.0
}

pend_l = ____
pend_r = ____
pend_m = ____
pend_com = {
    0.0, 0.0, pend_l
}

-- build model

bodies = {
    cart     = {mass = ___, com = ___, inertia = ___},
    pendulum = {mass = ___, com = ___, inertia = ___},
}

meshes = {
    cart = {
        name = "cart",
        dimensions = {cart_w, cart_d, cart_h},
        color = {0.0, 1.0, 0.0},
        mesh_center = {0, 0, 0},
        translate = {0, 0, 0},
        rotate = {0.0, 0.0, 0.0},
        src = "unit_cube.obj",
    },
    cart_com = {
        name = "cart_com",
        dimensions = {0.05, 0.05, 0.05},
        color = {0.0, 0.0, 0.0},
        mesh_center = cart_com,
        translate = {0, 0, 0},
        rotate = {0.0, 0.0, 0.0},
        src = "unit_cube.obj",
    },
    link = {
        name = "link",
        dimensions = {0.02, 0.02, pend_l},
        color = {0.5, 0.5, 0.0},
        mesh_center = {0, 0, 0},
        translate = {0, 0, pend_l/2.0},
        rotate = {0.0, 0.0, 0.0},
        src = "unit_cube.obj",
    },
    pendulum = {
        name = "pendulum",
        dimensions = {pend_r, pend_r, pend_r},
        color = {1.0, 0.0, 0.0},
        mesh_center = {0, 0, 0},
        translate = {0, 0, pend_l},
        rotate = {0.0, 0.0, 0.0},
        src = "unit_sphere_medres.obj",
    },
    pend_com = {
        name = "pend_com",
        dimensions = {0.05, 0.05, 0.05},
        color = {0.0, 0.0, 0.0},
        mesh_center = pend_com,
        translate = {0, 0, 0},
        rotate = {0.0, 0.0, 0.0},
        src = "unit_cube.obj",
    },
}

joints = {
    fixed = {},
    trans_x = {
        { _, _, _, _, _, _},
    },
    rot_y = {
        { _, _, _, _, _, _},
    },
}

model = {
    gravity = { 0., 0., -9.81},

    configuration = {
        axis_front = { 1.,  0.,  0.},
        axis_right = { 0., 1.,  0.},
        axis_up    = { 0.,  0.,  1.}
    },

    frames = {
        -- acctuated cart
        {
            name = "___",
            parent = "___",
            body = bodies.___,
            joint = joints.___,
            joint_frame = {
                r = {0.5, 0.5, 0.7},
                E = {
                    {1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {0.0, 0.0, 1.0},
                }
            },
            visuals = {
                meshes.cart,
                meshes.cart_com,
            }
        },
        -- unactuated pendulum
        {
            name = "___",
            parent = "___",
            body = bodies.___,
            joint = joints.___,
            joint_frame = {
                r = {0.0, 0.0, 0.0},
                E = {
                    {1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {0.0, 0.0, 1.0},
                }
            },
            visuals = {
                meshes.pendulum,
                meshes.pend_com,
                meshes.link,
            }
        },
        -- dummy frame for curve plotting
        {
            name = "pendulum_com",
            parent = "pendulum",
            joint = joints.fixed,
            joint_frame = {
                r = pend_com,
                E = {
                    {1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {0.0, 0.0, 1.0},
                }
            },
            visuals = {
            }
        },
    }
}

return model
