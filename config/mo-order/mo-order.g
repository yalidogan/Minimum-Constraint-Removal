Include: <../base-walls-min.g>

egoJoint(world){ Q:[0.0 0.0 0.1] } # works
ego(egoJoint) {
    Q:[-1, 0, 0], shape:ssCylinder, size:[.2 .2 .02], color:[0.96875 0.7421875 0.30859375], logical:{agent}, limits: [-4 4 -4 4], sampleUniform:.5,
    joint:transXY, contact: 1
}
##############################################################################################################################
wall1 (world){ shape:ssBox, Q:[-1.1 1 0.1], size:[1.8 .1 0.2 .04] , color:[0.6953 0.515625 .453125], contact: 1,  logical={ static} }
wall2 (world){ shape:ssBox, Q:[1.1 -1 0.1], size:[1.8 .1 0.2 .04] , color:[0.6953 0.515625 .453125], contact: 1,  logical={ static } }
##############################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(-.8 1.5 .1)",  size:[.2 .4 .2 .02], color:[1 0 0 .3], contact:0, joint:rigid,  logical={ goal } }
goal2 (floor){ shape:ssBox, Q:"t(-1.2 1.5 .1)", size:[.2 .4 .2 .02], color:[0 1 0 .3], contact:0, joint:rigid,  logical={ goal } }
goal3 (floor){ shape:ssBox, Q:"t(-1.6 1.5 .1)", size:[.2 .4 .2 .02], color:[0 0 1 .3], contact:0, joint:rigid,  logical={ goal } }
##############################################################################################################################
obj1Joint(world){ Q:[.8 -1.5 0.1] }
obj1(obj1Joint) { type:ssBox size:[.2 .4 .2 .02] color:[1. 0. 0.],  logical={ movable_go }, joint:rigid, contact: 1 }

obj2Joint(world){ Q:[1.2 -1.5 0.1] }
obj2(obj2Joint) { type:ssBox size:[.2 .4 .2 .02] color:[0. 1. 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj3Joint(world){ Q:[1.6 -1.5 0.1] }
obj3(obj3Joint) { type:ssBox size:[.2 .4 .2 .02] color:[0. 0. 1],  logical={ movable_go }, joint:rigid, contact: 1 }

##############################################################################################################################

