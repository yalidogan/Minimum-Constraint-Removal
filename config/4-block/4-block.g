Include: <../base-walls-min.g>
#########################################################################################################################
wall2_h (world){ shape:ssBox, Q:"t(1 -.75 0.1)", size:[.1 2.5 0.2 .02], color:[0.6953 0.515625 .453125],contact: 1 }
#########################################################################################################################
egoJoint(world){Q:[0 0 0.1]  }
ego(egoJoint) {
    shape:ssCylinder, Q:[-1.3 -1.3 0], size:[.2 .2 .02], color:[0.96875 0.7421875 0.30859375], logical:{agent}, limits: [-4 4 -4 4], sampleUniform:.5,
    joint:transXY, contact: 1
}
#########################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(1.6 -0.5 .2)", size:[.3 .3 0.2 .02], color:[0 0 1 0.3], contact:0, logical:{goal} }

#########################################################################################################################
obj1Joint(world){ Q:[0.0 0.0 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, Q:[-1.5 1.5 .0], size:[.3 .3 0.2 .02], logical={ movable_go }  nomass:1, color:[0 0 1.0],
    joint:rigid, friction:.1  contact: 1
}
#########################################################################################################################
obs1Joint(world){ Q:[0.0 0.0 0.1] } # works
obs1(obs1Joint) {
    shape:ssBox, Q:[-1.2 -0.3 .0], size:[1 0.2 .2 .02], logical={ movable_o }, nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1,  contact: 1
}

obs2Joint(world){ Q:[0.0 0.0 0.1] } # works
obs2(obs2Joint) {
    shape:ssBox, Q:[-0.75 -1.2 .0], size:[.2 1 .2 .02], logical={ movable_o }, nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1,  contact: 1
}

obs3Joint(world){ Q:[0.0 0.0 0.1] } # works
obs3(obs3Joint) {
    shape:ssBox, Q:[-0.5 -1.3 .0], size:[.2 1 .2 .02], logical={ movable_o }, nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1,  contact: 1
}

obs4Joint(world){ Q:[0.0 0.0 0.1] } # works
obs4(obs4Joint) {
    shape:ssBox, Q:[-1.4 -0.75 .0], size:[1 0.2 .2 .02], logical={ movable_o } ,nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1,  contact: 1
}

