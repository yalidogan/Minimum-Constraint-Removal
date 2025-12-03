Include: <../base-walls-min.g>

egoJoint(world){ Q:[0.0 0.0 0.1] } # works
ego(egoJoint) {
    Q:[1.6, -1.6, 0], shape:ssCylinder, size:[.2 .18 .02], color:[0.96875 0.7421875 0.30859375], logical:{agent}, limits: [-4 4 -4 4], sampleUniform:1,
    joint:transXY, contact: 1
}

##############################################################################################################################
#block_1 (floor){ Q:[0 0 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
#block_2 (floor){ Q:[.4 0 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_1 (floor){ Q:[.8 0 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_2 (floor){ Q:[.8 .4 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_3 (floor){ Q:[1.2 .4 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
#block_6 (floor){ Q:[1.6 .8 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_4 (floor){ Q:[-.4 .4 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_5 (floor){ Q:[-.4 .8 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_6 (floor){ Q:[-.4 1.2 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_7 (floor){ Q:[.8 .8 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_8 (floor){ Q:[-1.6 .8 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_9 (floor){ Q:[-1.6 .4 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
#block_10 (floor){ Q:[-1.2 -.4 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_10 (floor){ Q:[1.6 -.8 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
#block_12 (floor){ Q:[.4 -1.2 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
#block_16 (floor){ Q:[.8 -.8 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
block_11 (floor){ Q:[-1.6 -.8 0.1], shape:ssBox, size:[0.4, 0.4, 0.2, 0.01], color:[0.6953, 0.515625, 0.453125], logical:{static}, contact:1, joint:rigid}
##############################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(-1 -1.3 .1)", size:[.3 .3 .2 .02], color:[1. 0 0 .3], contact:0, joint:rigid}
goal2 (floor){ shape:ssBox, Q:"t(-0.4 -1.7 .1)", size:[.3 .3 .2 .02], color:[0 1 0 0.3], contact:0, joint:rigid }
goal3 (floor){ shape:ssBox, Q:"t(-1.6 -1.7 .1)", size:[.3 .3 .2 .02], color:[0 0 1 0.3], contact:0, joint:rigid }

##############################################################################################################################

obj1Joint(world){ Q:[1.7 .8 0.1] }
obj1(obj1Joint) { type:ssBox size:[.3 .3 .2 .02] color:[1. 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj2Joint(world){ Q:[.2 .8 0.1] }
obj2(obj2Joint) { type:ssBox size:[.3 .3 .2 .02] color:[0.2 0.8 0.3],  logical={ movable_go }, joint:rigid, contact: 1 }

obj3Joint(world){ Q:[.8 1.6 0.1] }
obj3(obj3Joint) { type:ssBox size:[.3 .3 .2 .02] color:[0. 0. 1.],  logical={ movable_go }, joint:rigid, contact: 1 }

##############################################################################################################################

