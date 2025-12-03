Include: <../base-walls-min.g>
##############################################################################################################################
wall1_v (world){ shape:ssBox, Q:"t(1.4 0.0 0.3)", size:[0.1 2.6 0.6 .02], color:[0.6953 0.515625 .453125], logical:{static},  contact: 1 }
wall2_h (world){ shape:ssBox, Q:"t(0.0 -1.25 0.3)", size:[2.65 .1 0.6 .02], color:[0.6953 0.515625 .453125], logical:{static},  contact: 1 }
wall4_h (world){ shape:ssBox, Q:"t(0.0 1.25 0.3)", size:[2.65 .1 0.6 .02], color:[0.6953 0.515625 .453125], logical:{static},  contact: 1 }
wall5_v (world){ shape:ssBox, Q:"t(-1.4 0.0 0.3)", size:[0.1 2.6 0.6 .02], color:[0.6953 0.515625 .453125], logical:{static},  contact: 1 }
##############################################################################################################################
egoJoint(world){ Q:[0 0.0 0.1] } # works
ego(egoJoint) {
    shape:ssCylinder, Q:[1.7 0.0 0], size:[.2 .2 .02], color:[0.96875 0.7421875 0.30859375], logical:{agent}, limits: [-4 4 -4 4], sampleUniform:1,
    joint:transXY, contact: 1
}
##############################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(-1.6 1.6 .2)", size:[.3 .3 .2 .02], color:[0 0 1 0.3], contact:0, logical:{goal} }
##############################################################################################################################
obj1Joint(world){ Q:[1.5 -1.5 0.1] } # works
obj1(obj1Joint) {type:ssBox size:[.3 .3 .2 .02] color:[0. 0. 1.],  logical={ movable_go }, joint:rigid, contact: 1 }