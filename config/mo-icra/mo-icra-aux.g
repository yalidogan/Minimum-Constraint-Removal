Include: <../base-walls-min-heatmap.g>
cam0(world){ Q:"t(0 0 10) d(180 1 0 0)" shape:camera width:300 height:300}
#########################################################################################################################egoJoint(world){Q:[0 0 -0.3]  }
egoJoint(world){Q:[0 0 0.2]  }
ego(egoJoint) {
    shape:ssCylinder, Q:[-1, 0, 0], size:[.4 .02 .02], color:[1 1 0]
}
##############################################################################################################################
goal1 (floor){ shape:ssCylinder, Q:"t(-1.7 1 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal2 (floor){ shape:ssCylinder, Q:"t(-1.7 0.5 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal3 (floor){ shape:ssCylinder, Q:"t(-1.7 0 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal4  (floor){ shape:ssCylinder, Q:"t(-1.7 -0.5 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal5  (floor){ shape:ssCylinder, Q:"t(-1.7 -1 .15)", size:[.3 .15 .05],   color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
##############################################################################################################################
goal6 (floor){ shape:ssCylinder, Q:"t(-.6 1 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal7 (floor){ shape:ssCylinder, Q:"t(-1 .6 .15)", size:[.3 .15 .05],   color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal8 (floor){ shape:ssCylinder, Q:"t(-1 -.6 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal9 (floor){ shape:ssCylinder, Q:"t(-.6 -1 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal10  (floor){ shape:ssCylinder, Q:"t(-1.2 0 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
##############################################################################################################################
goal11  (floor){ shape:ssCylinder, Q:"t(-.1 1 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal12  (floor){ shape:ssCylinder, Q:"t(-.1 .5 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal13 (floor){ shape:ssCylinder, Q:"t(-.1 0 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal14 (floor){ shape:ssCylinder, Q:"t(.4 .8 .15)", size:[.3 .15 .05],   color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal15 (floor){ shape:ssCylinder, Q:"t(.4 .3 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal16  (floor){ shape:ssCylinder, Q:"t(-.1 -.5 .15)", size:[.3 .15 .05],   color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal17  (floor){ shape:ssCylinder, Q:"t(-.1 -1 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal18  (floor){ shape:ssCylinder, Q:"t(.4 -.3 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal19  (floor){ shape:ssCylinder, Q:"t(.55 -1 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
##############################################################################################################################
goal20  (floor){ shape:ssCylinder, Q:"t(1.25 1 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal21  (floor){ shape:ssCylinder, Q:"t(1.7 -1 .15)", size:[.3 .15 .05],   color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal22  (floor){ shape:ssCylinder, Q:"t(.9 -1 .15)", size:[.3 .15 .05],   color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal23  (floor){ shape:ssCylinder, Q:"t(1.5 .3 .15)", size:[.3 .15 .05],  color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal24  (floor){ shape:ssCylinder, Q:"t(1.6 -.4 .15)", size:[.3 .15 .05], color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal25  (floor){ shape:ssCylinder, Q:"t(1 -.4 .15)", size:[.3 .15 .05],   color:[0 0 1], contact:0, logical:{goal}, joint:rigid }
goal26  (floor){ shape:ssCylinder, Q:"t(1.1 .3 .15)", size:[.3 .15 .05],   color:[0 0 1], contact:0, logical:{goal}, joint:rigid }

obj1Joint(world){ Q:[-1.5 -1.3 .15] }
obj1(obj1Joint) { type:ssCylinder size:[.3 .15 .05] color:[1. 0. 0.],  logical={ movable_go }, joint:rigid, contact: 1 }
obj1_cam(obj1){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj2Joint(world){ Q:[-1.5 -0.8 .15] }
obj2(obj2Joint) { type:ssCylinder size:[.3 .15 .05] color:[1. 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj2_cam(obj2){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj3Joint(world){ Q:[-1.5 -0.3 .15] }
obj3(obj3Joint) { type:ssCylinder size:[.3 .15 .05] color:[1. 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj3_cam(obj3){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}
##############################################################################################################################
obj4Joint(world){ Q:[-.5 -1.3 .15] }
obj4(obj4Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj4_cam(obj4){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj5Joint(world){ Q:[-.9 -1 .15] }
obj5(obj5Joint) { type:ssCylinder size:[.3 .15 .05] color:[1. 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj5_cam(obj5){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj6Joint(world){ Q:[-.9 -.5 .15] }
obj6(obj6Joint) { type:ssCylinder size:[.3 .15 .05] color:[1. 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj6_cam(obj6){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj7Joint(world){ Q:[-.5 -.3 .15] }
obj7(obj7Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj7_cam(obj7){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}
##############################################################################################################################
obj8Joint(world){ Q:[.1 -1.3 .15] }
obj8(obj8Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj8_cam(obj8){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj9Joint(world){ Q:[.1 -.8 .15] }
obj9(obj9Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj9_cam(obj9){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj10Joint(world){ Q:[.1 -.3 .15] }
obj10(obj10Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj10_cam(obj10){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj11Joint(world){ Q:[.6 -1 .15] }
obj11(obj11Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj11_cam(obj11){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj12Joint(world){ Q:[.6 -.3 .15] }
obj12(obj12Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj12_cam(obj12){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################
obj13Joint(world){ Q:[1.35 -1.3 .15] }
obj13(obj13Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj13_cam(obj13){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj14Joint(world){ Q:[1.6 -.3 .15] }
obj14(obj14Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj14_cam(obj14){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj15Joint(world){ Q:[1.1 -.3 .15] }
obj15(obj15Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj15_cam(obj15){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj16Joint(world){ Q:[1.35 -.8 .15] }
obj16(obj16Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj16_cam(obj16){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################
obj17Joint(world){ Q:[-.7 1.4 .15] }
obj17(obj17Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj17_cam(obj17){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj18Joint(world){ Q:[-.2 1 .15] }
obj18(obj18Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj18_cam(obj18){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj19Joint(world){ Q:[0 .6 .15] }
obj19(obj19Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj19_cam(obj19){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj20Joint(world){ Q:[-.3 .3 .15] }
obj20(obj20Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj20_cam(obj20){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj21Joint(world){ Q:[-.7 .6 .15] }
obj21(obj21Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj21_cam(obj21){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj22Joint(world){ Q:[0 1.4 .15] }
obj22(obj22Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj22_cam(obj22){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################
obj23Joint(world){ Q:[.5 1.4 .15] }
obj23(obj23Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj23_cam(obj23){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj24Joint(world){ Q:[.5 .9 .15] }
obj24(obj24Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj24_cam(obj24){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj25Joint(world){ Q:[.5 .3 .15] }
obj25(obj25Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj25_cam(obj25){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj26Joint(world){ Q:[.5 0 .15] }
obj26(obj26Joint) { type:ssCylinder size:[.3 .15 .05] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj26_cam(obj26){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

