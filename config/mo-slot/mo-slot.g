Include: <../base-walls-min.g>
##############################################################################################################################
egoJoint(world){ Q:[0.0 0.0 0.1] } # works
ego(egoJoint) {
    Q:[-1.6, 1.6, 0], shape:ssCylinder, size:[.2 .2 .02], color:[0.96875 0.7421875 0.30859375], logical:{agent}, limits: [-4 4 -4 4], sampleUniform:1,
    joint:transXY, contact: 1
}
##############################################################################################################################
wall1 (world){ shape:ssBox, Q:[.8 -1.5 0.2], size:[.1 1 0.4 .04] , color:[0.6953 0.515625 .453125], contact: 1 }
wall2 (world){ shape:ssBox, Q:[1.7 -.9 0.2], size:[0.7 .1 0.4 .04] , color:[0.6953 0.515625 .453125], contact: 1 }
##############################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(1.2 -1.5 .0)", size:[.2 .6 .2 .02], color:[1 0 0 0.3], contact:0, logical:{goal} }
goal2 (floor){ shape:ssBox, Q:"t(1.6 -1.5 .0)", size:[.2 .6 .2 .02], color:[0. 0 1 0.3], contact:0, logical:{goal} }
##############################################################################################################################
obj1Joint(world){ Q:[-1.6 -1.5 0.1] }
obj1(obj1Joint) { type:ssBox size:[.2 .6 .2 .02] color:[1. 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj2Joint(world){ Q:[1.6 1.5 0.1] }
obj2(obj2Joint) { type:ssBox size:[.2 .6 .2 .02] color:[0. 0. 1.],  logical={ movable_go }, joint:rigid, contact: 1 }
##############################################################################################################################
obs1Joint(world){ Q:[1.1 -.85 0.1] }
obs1(obs1Joint) { type:ssBox size:[.4 .2 .2 .02] color:[1. 1. 1],  logical={ movable_o }, joint:rigid, contact: 1 }
