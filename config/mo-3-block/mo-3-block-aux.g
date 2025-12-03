Include: <../base-walls-min-heatmap.g>
cam0(world){ Q:"t(0 0 10) d(180 1 0 0)" shape:camera width:300 height:300}
#########################################################################################################################
egoJoint(world){Q:[0 0 0.2]  }
ego(egoJoint) {
    shape:ssCylinder, Q:[1.6, -1.6, 0], size:[.4 .02 .02], color:[1 1 0]
}
##############################################################################################################################
#block_1 (floor){ Q:[0 0 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
#block_2 (floor){ Q:[.4 0 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_3 (floor){ Q:[.8 0 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_4 (floor){ Q:[.8 .4 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_5 (floor){ Q:[1.2 .4 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
#block_6 (floor){ Q:[1.6 .8 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_7 (floor){ Q:[-.4 .4 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_8 (floor){ Q:[-.4 .8 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_9 (floor){ Q:[-.4 1.2 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_10 (floor){ Q:[.8 .8 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_11 (floor){ Q:[-1.6 .8 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_12 (floor){ Q:[-1.6 .4 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
#block_13 (floor){ Q:[-1.2 -.4 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_14 (floor){ Q:[1.6 -.8 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
#block_15 (floor){ Q:[.4 -1.2 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
#block_16 (floor){ Q:[.8 -.8 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
block_17 (floor){ Q:[-1.6 -.8 0.2], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1 ,0 ,0 ], contact:1}
##############################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(-1 -1.3 .1)", size:[.3 .3 .2 .02], color:[0 0 1 0], contact:0, joint:rigid}
goal2 (floor){ shape:ssBox, Q:"t(-0.4 -1.7 .1)", size:[.3 .3 .2 .02], color:[0 0 1 0], contact:0, joint:rigid }
goal3 (floor){ shape:ssBox, Q:"t(-1.6 -1.7 .1)", size:[.3 .3 .2 .02], color:[0 0 1 0], contact:0, joint:rigid }
##############################################################################################################################
obj1Joint(world){ Q:[1.7 .8 0.1] }
obj1(obj1Joint) { type:ssBox size:[.3 .3 .2 .02] color:[1 0 0],  logical={ object }, joint:rigid, contact: 1 }
obj1_cam(obj1){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################
obj2Joint(world){ Q:[.2 .8 0.1] }
obj2(obj2Joint) { type:ssBox size:[.3 .3 .2 .02] color:[1. 0. 0],  logical={ object }, joint:rigid, contact: 1 }
obj2_cam(obj2){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################
obj3Joint(world){ Q:[.8 1.6 0.1] }
obj3(obj3Joint) { type:ssBox size:[.3 .3 .2 .02] color:[1 0 0],  logical={ object }, joint:rigid, contact: 1 }
obj3_cam(obj3){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################

