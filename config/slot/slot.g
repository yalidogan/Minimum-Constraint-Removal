Include: <../base-walls-min.g>
##############################################################################################################################
wall_h (floor){ shape:ssBox, Q:"t(1.09 -0.9 0.1)", size:[1.82 .1 0.2 .02], color:[0.6953 0.515625 .453125],contact: 1, logical:{static} }
wall1_v (floor){ shape:ssBox, Q:"t(0.2 1.5 0.1)", size:[0.1 1 0.2 .02], color:[0.6953 0.515625 .453125], contact: 1 , logical:{static}}
wall2_v (floor){ shape:ssBox, Q:"t(0.2 -0.4 0.1)", size:[0.1 1 0.2 .02], color:[0.6953 0.515625 .453125], contact: 1 , logical:{static}}
##############################################################################################################################
egoJoint(world){ Q:[0 0.0 0.1] } # works
ego(egoJoint) {
    shape:ssCylinder, Q:[1.6 -1.6 0], size:[.2 .2 .02], color:[0.96875 0.7421875 0.30859375], logical:{agent}, limits: [-4 4 -4 4],
    joint:transXY, contact: 1
}
##############################################################################################################################
goal1(floor){ shape:ssBox, Q:"t(1.6 -.4 .2)", size:[.3 .3 .2 .02], color:[0 0 1 .3], contact:0, logical:{goal} }
##############################################################################################################################
obj1Joint(world){ Q:[0 0 0.1] } # works
obj1(obj1Joint) { type:ssBox size:[.3 .3 .2 .02] Q:"t(-1.6 1.6  0)" color:[0 0 1],  logical={ movable_go }, joint:rigid, contact: 1 }
##############################################################################################################################
obs1Joint(world){ Q:[0.0 0.0 0.1] } # works
obs1(obs1Joint) {
    shape:ssBox, Q:[0.0 0.6 .0], size:[.2 .5 .2 .02], logical={ movable_o } ,  color:[1 1 1],
    joint:rigid,  contact: 1
}

obs2Joint(world){ Q:[0.0 0.0 0.1] } # works
obs2(obs2Joint) {
    shape:ssBox, Q:[-1 1.4 .0], size:[.2 .6 .2 .02], logical={ movable_o } ,  color:[1 1.0 1],
    joint:rigid,  contact: 1
}

obs3Joint(world){ Q:[0.0 0.0 0.1] } # works
obs3(obs3Joint) {
    shape:ssBox, Q:[-0.4 1.4 .0], size:[.2 1 .2 .02], logical={ movable_o } ,  color:[1.0 1 1],
    joint:rigid,  contact: 1
}

obs4Joint(world){ Q:[0.0 0.0 0.1] } # works
obs4(obs4Joint) {
    shape:ssBox, Q:[-1.5 -1.4 .0], size:[.2 0.5 .2 .02], logical={ movable_o } ,  color:[1 1 1],
    joint:rigid,  contact: 1
}

obs5Joint(world){ Q:[0.0 0.0 0.1] } # works
obs5(obs5Joint) {
    shape:ssBox, Q:[0.4 -1.45 .0], size:[.2 .8 .2 .02], logical={ movable_o } ,  color:[1 1 1],
    joint:rigid, contact: 1
}
