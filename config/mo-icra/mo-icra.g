Include: <../base-walls-min.g>
##############################################################################################################################
egoJoint(world){ Q:[0.0 0.0 0.1] } # works
ego(egoJoint) {
    Q:[-1, 0, 0], shape:ssCylinder, size:[.2 .12 .02], color:[0.96875 0.7421875 0.30859375], logical:{agent}, limits: [-4 4 -4 4], sampleUniform:.5,
    joint:transXY, contact: 1
}
##############################################################################################################################
goal1 (floor){ shape:ssCylinder, Q:"t(-1.7 1 .15)", size:[.3 .15 .05], color:[1 0 0 .1], contact:0, logical:{goal}, joint:rigid }
goal2 (floor){ shape:ssCylinder, Q:"t(-1.7 0.5 .15)", size:[.3 .15 .05], color:[.6 0 0 .1], contact:0, logical:{goal}, joint:rigid }
goal3 (floor){ shape:ssCylinder, Q:"t(-1.7 0 .15)", size:[.3 .15 .05], color:[.2 0 0 .1], contact:0, logical:{goal}, joint:rigid }
goal4  (floor){ shape:ssCylinder, Q:"t(-1.7 -0.5 .15)", size:[.3 .15 .05], color:[1 0 0 .1], contact:0, logical:{goal}, joint:rigid }
goal5  (floor){ shape:ssCylinder, Q:"t(-1.7 -1 .15)", size:[.3 .15 .05],   color:[.8 0 .0 .1], contact:0, logical:{goal}, joint:rigid }
##############################################################################################################################
goal6 (floor){ shape:ssCylinder, Q:"t(-.6 1 .15)", size:[.3 .15 .05], color:[0 1 0 .1], contact:0, logical:{goal}, joint:rigid }
goal7 (floor){ shape:ssCylinder, Q:"t(-1 .6 .15)", size:[.3 .15 .05],   color:[0 .7 0 .1], contact:0, logical:{goal}, joint:rigid }
goal8 (floor){ shape:ssCylinder, Q:"t(-1 -.6 .15)", size:[.3 .15 .05],  color:[0 .5 0 .1], contact:0, logical:{goal}, joint:rigid }
goal9 (floor){ shape:ssCylinder, Q:"t(-.6 -1 .15)", size:[.3 .15 .05],  color:[0 .3 0 .1], contact:0, logical:{goal}, joint:rigid }
goal10  (floor){ shape:ssCylinder, Q:"t(-1.2 0 .15)", size:[.3 .15 .05], color:[0 .3 0 .1], contact:0, logical:{goal}, joint:rigid }
##############################################################################################################################
goal11  (floor){ shape:ssCylinder, Q:"t(-.1 1 .15)", size:[.3 .15 .05], color:[0 0 1 .1], contact:0, logical:{goal}, joint:rigid }
goal12  (floor){ shape:ssCylinder, Q:"t(-.1 .5 .15)", size:[.3 .15 .05],  color:[0 0 .8 .1], contact:0, logical:{goal}, joint:rigid }
goal13 (floor){ shape:ssCylinder, Q:"t(-.1 0 .15)", size:[.3 .15 .05],  color:[0 0 .6 .1], contact:0, logical:{goal}, joint:rigid }
goal14 (floor){ shape:ssCylinder, Q:"t(.4 .8 .15)", size:[.3 .15 .05],   color:[0 0 .4 .1], contact:0, logical:{goal}, joint:rigid }
goal15 (floor){ shape:ssCylinder, Q:"t(.4 .3 .15)", size:[.3 .15 .05],  color:[0 0 .1 .1], contact:0, logical:{goal}, joint:rigid }
goal16  (floor){ shape:ssCylinder, Q:"t(-.1 -.5 .15)", size:[.3 .15 .05],   color:[0 0 .8 .1], contact:0, logical:{goal}, joint:rigid }
goal17  (floor){ shape:ssCylinder, Q:"t(-.1 -1 .15)", size:[.3 .15 .05],  color:[0 0 .5 .1], contact:0, logical:{goal}, joint:rigid }
goal18  (floor){ shape:ssCylinder, Q:"t(.4 -.3 .15)", size:[.3 .15 .05],  color:[0 0 .4 .1], contact:0, logical:{goal}, joint:rigid }
goal19  (floor){ shape:ssCylinder, Q:"t(.55 -1 .15)", size:[.3 .15 .05],  color:[0 0 .2 .1], contact:0, logical:{goal}, joint:rigid }
##############################################################################################################################
goal20  (floor){ shape:ssCylinder, Q:"t(1.25 1 .15)", size:[.3 .15 .05], color:[0 1 1 .1], contact:0, logical:{goal}, joint:rigid }
goal21  (floor){ shape:ssCylinder, Q:"t(1.7 -1 .15)", size:[.3 .15 .05],   color:[0 .7 .7 .1], contact:0, logical:{goal}, joint:rigid }
goal22  (floor){ shape:ssCylinder, Q:"t(.9 -1 .15)", size:[.3 .15 .05],   color:[0 .5 .5 .1], contact:0, logical:{goal}, joint:rigid }
goal23  (floor){ shape:ssCylinder, Q:"t(1.5 .3 .15)", size:[.3 .15 .05],  color:[0 .3 .3 .1], contact:0, logical:{goal}, joint:rigid }
goal24  (floor){ shape:ssCylinder, Q:"t(1.6 -.4 .15)", size:[.3 .15 .05], color:[1 1 1 .1], contact:0, logical:{goal}, joint:rigid }
goal25  (floor){ shape:ssCylinder, Q:"t(1 -.4 .15)", size:[.3 .15 .05],   color:[.8 .8 .8 .1], contact:0, logical:{goal}, joint:rigid }
goal26  (floor){ shape:ssCylinder, Q:"t(1.1 .3 .15)", size:[.3 .15 .05],   color:[.6 .6 .6 .1], contact:0, logical:{goal}, joint:rigid }

##############################################################################################################################
obj1Joint(world){ Q:[-1.5 -1.3 .15] }
obj1(obj1Joint) { type:ssCylinder size:[.3 .15 .05] color:[1. 0. 0.],  logical={ movable_go }, joint:rigid, contact: 1 }

obj2Joint(world){ Q:[-1.5 -0.8 .15] }
obj2(obj2Joint) { type:ssCylinder size:[.3 .15 .05] color:[.9 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj3Joint(world){ Q:[-1.5 -0.3 .15] }
obj3(obj3Joint) { type:ssCylinder size:[.3 .15 .05] color:[.7 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj4Joint(world){ Q:[-.5 -1.3 .15] }
obj4(obj4Joint) { type:ssCylinder size:[.3 .15 .05] color:[.5 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj5Joint(world){ Q:[-.9 -1 .15] }
obj5(obj5Joint) { type:ssCylinder size:[.3 .15 .05] color:[.3 .0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
##############################################################################################################################

obj6Joint(world){ Q:[-.9 -.5 .15] }
obj6(obj6Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 1 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj7Joint(world){ Q:[-.5 -.3 .15] }
obj7(obj7Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 .8 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj8Joint(world){ Q:[.1 -1.3 .15] }
obj8(obj8Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 .7 0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj9Joint(world){ Q:[.1 -.8 .15] }
obj9(obj9Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 .5 .0],  logical={ movable_go }, joint:rigid, contact: 1 }

obj10Joint(world){ Q:[.1 -.3 .15] }
obj10(obj10Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 .3 0],  logical={ movable_go }, joint:rigid, contact: 1 }
##############################################################################################################################
obj11Joint(world){ Q:[.6 -1 .15] }
obj11(obj11Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 0 1],  logical={ movable_go }, joint:rigid, contact: 1 }

obj12Joint(world){ Q:[.6 -.3 .15] }
obj12(obj12Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 0 .9],  logical={ movable_go }, joint:rigid, contact: 1 }

obj13Joint(world){ Q:[1.35 -1.3 .15] }
obj13(obj13Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 0 .8],  logical={ movable_go }, joint:rigid, contact: 1 }

obj14Joint(world){ Q:[1.6 -.3 .15] }
obj14(obj14Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 .0 .7],  logical={ movable_go }, joint:rigid, contact: 1 }

obj15Joint(world){ Q:[1.1 -.3 .15] }
obj15(obj15Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 .0 .6],  logical={ movable_go }, joint:rigid, contact: 1 }

obj16Joint(world){ Q:[1.35 -.8 .15] }
obj16(obj16Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 .0 .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj17Joint(world){ Q:[-.7 1.4 .15] }
obj17(obj17Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 0 .4],  logical={ movable_go }, joint:rigid, contact: 1 }

obj18Joint(world){ Q:[-.2 1 .15] }
obj18(obj18Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 0 .3],  logical={ movable_go }, joint:rigid, contact: 1 }

obj19Joint(world){ Q:[0 .6 .15] }
obj19(obj19Joint) { type:ssCylinder size:[.3 .15 .05] color:[0 0 .2],  logical={ movable_go }, joint:rigid, contact: 1 }
##############################################################################################################################
obj20Joint(world){ Q:[-.3 .3 .15] }
obj20(obj20Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 1],  logical={ movable_go }, joint:rigid, contact: 1 }

obj21Joint(world){ Q:[-.7 .6 .15] }
obj21(obj21Joint) { type:ssCylinder size:[.3 .15 .05] color:[.9 0 .9],  logical={ movable_go }, joint:rigid, contact: 1 }

obj22Joint(world){ Q:[0 1.4 .15] }
obj22(obj22Joint) { type:ssCylinder size:[.3 .15 .05] color:[.8 0 .8],  logical={ movable_go }, joint:rigid, contact: 1 }

obj23Joint(world){ Q:[.5 1.4 .15] }
obj23(obj23Joint) { type:ssCylinder size:[.3 .15 .05] color:[.7 0 .7],  logical={ movable_go }, joint:rigid, contact: 1 }

obj24Joint(world){ Q:[.5 .9 .15] }
obj24(obj24Joint) { type:ssCylinder size:[.3 .15 .05] color:[.6 0 .6],  logical={ movable_go }, joint:rigid, contact: 1 }

obj25Joint(world){ Q:[.5 .3 .15] }
obj25(obj25Joint) { type:ssCylinder size:[.3 .15 .05] color:[.5 .0 .5],  logical={ movable_go }, joint:rigid, contact: 1 }

obj26Joint(world){ Q:[.5 .0 .15] }
obj26(obj26Joint) { type:ssCylinder size:[.3 .15 .05] color:[.4 .0 .4],  logical={ movable_go }, joint:rigid, contact: 1 }


