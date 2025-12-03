Include: <../base-walls-min.g>
##############################################################################################################################
egoJoint(world){ Q:[0.0 0.0 0.1] } # works
ego(egoJoint) {
    Q:[-1, 0, 0], shape:ssCylinder, size:[.2 .15 .02], color:[0.96875 0.7421875 0.30859375], logical:{agent}, limits: [-4 4 -4 4], sampleUniform:.5,
    joint:transXY, contact: 1
}
##############################################################################################################################
goal1 (floor){ shape:ssCylinder, Q:"t(-1.2 1 .15)", size:[.2 .2 .05], color:[0 0 1 .1], contact:0, logical:{goal}, joint:rigid }
goal2 (floor){ shape:ssCylinder, Q:"t(-.4 1 .15)", size:[.2 .2 .05], color:[0 1 0 .1], contact:0, logical:{goal}, joint:rigid }
goal3 (floor){ shape:ssCylinder, Q:"t(.4 1 .15)", size:[.2 .2 .05], color:[1 0 0 .1], contact:0, logical:{goal}, joint:rigid }
goal4 (floor){ shape:ssCylinder, Q:"t(1.2 1 .15)", size:[.2 .2 .05], color:[0 1 1 .1], contact:0, logical:{goal}, joint:rigid }
goal5 (floor){ shape:ssCylinder, Q:"t(-1.2 -1 .15)", size:[.2 .2 .05], color:[0 0 .5 .1], contact:0, logical:{goal}, joint:rigid }
goal6 (floor){ shape:ssCylinder, Q:"t(-.4 -1 .15)", size:[.2 .2 .05], color:[0 .5 0 .1], contact:0, logical:{goal}, joint:rigid }
goal7 (floor){ shape:ssCylinder, Q:"t(.4 -1 .15)", size:[.2 .2 .05], color:[.5 0 0 .1], contact:0, logical:{goal}, joint:rigid }
goal8 (floor){ shape:ssCylinder, Q:"t(1.2 -1 .15)", size:[.2 .2 .05], color:[.8 .5 .5 .1], contact:0, logical:{goal}, joint:rigid }
goal9 (floor){ shape:ssCylinder, Q:"t(-1.2 -1 .15)", size:[.2 .2 .05], color:[.5 .8 .5 .1], contact:0, logical:{goal}, joint:rigid }
goal10 (floor){ shape:ssCylinder, Q:"t(-.4 -1 .15)", size:[.2 .2 .05], color:[.5 .5 .8 .1], contact:0, logical:{goal}, joint:rigid }
goal11 (floor){ shape:ssCylinder, Q:"t(.4 -1 .15)", size:[.2 .2 .05], color:[.8 .5 .8 .1], contact:0, logical:{goal}, joint:rigid }
goal12 (floor){ shape:ssCylinder, Q:"t(1.2 -1 .15)", size:[.2 .2 .05], color:[.8 .8 .5 .1], contact:0, logical:{goal}, joint:rigid }
goal13 (floor){ shape:ssCylinder, Q:"t(-1.2 -1 .15)", size:[.2 .2 .05], color:[0 0 .25 .1], contact:0, logical:{goal}, joint:rigid }
goal14 (floor){ shape:ssCylinder, Q:"t(-.4 -1 .15)", size:[.2 .2 .05], color:[0 .25 0 .1], contact:0, logical:{goal}, joint:rigid }
goal15 (floor){ shape:ssCylinder, Q:"t(.4 -1 .15)", size:[.2 .2 .05], color:[.25 0 0 .1], contact:0, logical:{goal}, joint:rigid }
goal16 (floor){ shape:ssCylinder, Q:"t(1.2 -1 .15)", size:[.2 .2 .05], color:[0 .25 .25 .1], contact:0, logical:{goal}, joint:rigid }
goal17 (floor){ shape:ssCylinder, Q:"t(-1.2 -1 .15)", size:[.2 .2 .05], color:[0 0 .25 .1], contact:0, logical:{goal}, joint:rigid }
goal18 (floor){ shape:ssCylinder, Q:"t(-.4 -1 .15)", size:[.2 .2 .05], color:[0 .25 0 .1], contact:0, logical:{goal}, joint:rigid }
goal19 (floor){ shape:ssCylinder, Q:"t(.4 -1 .15)", size:[.2 .2 .05], color:[.25 0 0 .1], contact:0, logical:{goal}, joint:rigid }
goal20 (floor){ shape:ssCylinder, Q:"t(1.2 -1 .15)", size:[.2 .2 .05], color:[0 .25 .25 .1], contact:0, logical:{goal}, joint:rigid }
##############################################################################################################################

obj1Joint(world){ Q:[0 -1 0.15] }
obj1(obj1Joint) { type:ssCylinder size:[.3 .2 .05] color:[0. 0. 1.],  logical={ movable_go }, joint:rigid, contact: 1 }

obj2Joint(world){ Q:[0.8 -.8 0.15] }
obj2(obj2Joint) { type:ssCylinder size:[.3 .2 .05] color:[0. 1. 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj3Joint(world){ Q:[-0.6 -1.4 0.15] }
obj3(obj3Joint) { type:ssCylinder size:[.3 .2 .05] color:[1. 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj4Joint(world){ Q:[-1.2 -.8 0.15] }
obj4(obj4Joint) { type:ssCylinder size:[.3 .2 .05] color:[0 1. 1],  logical={ movable_go }, joint:rigid, contact: 1 }

##############################################################################################################################

obj5Joint(world){ Q:[0 1 0.15] }
obj5(obj5Joint) { type:ssCylinder size:[.3 .2 .05] color:[0. 0. .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj6Joint(world){ Q:[-0.8 .5 0.15] }
obj6(obj6Joint) { type:ssCylinder size:[.3 .2 .05] color:[0. .5 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj7Joint(world){ Q:[0.6 -.4 0.15] }
obj7(obj7Joint) { type:ssCylinder size:[.3 .2 .05] color:[.5 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj8Joint(world){ Q:[1.2 0 0.15] }
obj8(obj8Joint) { type:ssCylinder size:[.3 .2 .05] color:[.8 .5 .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj9Joint(world){ Q:[0 1 0.15] }
obj9(obj9Joint) { type:ssCylinder size:[.3 .2 .05] color:[.5 .8 .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj10Joint(world){ Q:[-0.8 .5 0.15] }
obj10(obj10Joint) { type:ssCylinder size:[.3 .2 .05] color:[.5 .5 .8],  logical={ movable_go }, joint:rigid, contact: 1 }

obj11Joint(world){ Q:[0.6 -.4 0.15] }
obj11(obj11Joint) { type:ssCylinder size:[.3 .2 .05] color:[.8 .5 .8],  logical={ movable_go }, joint:rigid, contact: 1 }

obj12Joint(world){ Q:[1.2 0 0.15] }
obj12(obj12Joint) { type:ssCylinder size:[.3 .2 .05] color:[.8 .8 .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj13Joint(world){ Q:[0 1 0.15] }
obj13(obj13Joint) { type:ssCylinder size:[.3 .2 .05] color:[.5 .8 .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj14Joint(world){ Q:[-0.8 .5 0.15] }
obj14(obj14Joint) { type:ssCylinder size:[.3 .2 .05] color:[.5 .5 .8],  logical={ movable_go }, joint:rigid, contact: 1 }

obj15Joint(world){ Q:[0.6 -.4 0.15] }
obj15(obj15Joint) { type:ssCylinder size:[.3 .2 .05] color:[.8 .5 .8],  logical={ movable_go }, joint:rigid, contact: 1 }

obj16Joint(world){ Q:[1.2 0 0.15] }
obj16(obj16Joint) { type:ssCylinder size:[.3 .2 .05] color:[.8 .8 .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj17Joint(world){ Q:[0 1 0.15] }
obj17(obj17Joint) { type:ssCylinder size:[.3 .2 .05] color:[.5 .8 .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj18Joint(world){ Q:[-0.8 .5 0.15] }
obj18(obj18Joint) { type:ssCylinder size:[.3 .2 .05] color:[.5 .5 .8],  logical={ movable_go }, joint:rigid, contact: 1 }

obj19Joint(world){ Q:[0.6 -.4 0.15] }
obj19(obj19Joint) { type:ssCylinder size:[.3 .2 .05] color:[.8 .5 .8],  logical={ movable_go }, joint:rigid, contact: 1 }

obj20Joint(world){ Q:[1.2 0 0.15] }
obj20(obj20Joint) { type:ssCylinder size:[.3 .2 .05] color:[.8 .8 .5],  logical={ movable_go }, joint:rigid, contact: 1 }



