Include: <../base-walls-min-heatmap.g>
cam0(world){ Q:"t(0 0 10) d(180 1 0 0)" shape:camera width:300 height:300}
##############################################################################################################################

egoJoint(world){Q:[0 0 0.2]  }
ego(egoJoint) {
    shape:ssCylinder, Q:[1.2 1.2, 0], size:[.4 .02 .02], color:[1 1 0]
}
##############################################################################################################################

wall1 (world){ shape:ssBox, Q:[1 1.8 0.2], size:[.1 .4 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }
wall2 (world){ shape:ssBox, Q:[1.8 1 0.2], size:[.4 .1 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }

wall3 (world){ shape:ssBox, Q:[.25 -.25 0.2 0.9305076 0 0 0.3662725], size:[2 .1 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }
wall4 (world){ shape:ssBox, Q:[-.25 .25 0.2 0.9305076 0 0 0.3662725], size:[2 .1 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }

wall5 (world){ shape:ssBox, Q:[1 -1.8 0.2], size:[.1 .4 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }
wall6 (world){ shape:ssBox, Q:[1.8 -1 0.2], size:[.4 .1 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }

wall7 (world){ shape:ssBox, Q:[-1 -1.8 0.2], size:[.1 .4 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }
wall8 (world){ shape:ssBox, Q:[-1.8 -1 0.2], size:[.4 .1 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }

wall9 (world){ shape:ssBox, Q:[-1 1.8 0.2], size:[.1 .4 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }
wall10(world){ shape:ssBox, Q:[-1.8 1 0.2], size:[.4 .1 0.4 .04] , color:[1 0 0], contact: 1, logical: {static} }

##############################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(1.6 -1.6 .1)",  size:[.25 .25 .2 .02], color:[0 0 1 ], contact:0, joint:rigid, logical: {goal} }
goal2 (floor){ shape:ssBox, Q:"t(-1.5 -1.5 .1)", size:[.25 .25 .2 .02], color:[0 0 1 ], contact:0, joint:rigid, logical: {goal} }
goal3 (floor){ shape:ssBox, Q:"t(-.3 -.3 .1)", size:[.25 .25 .2 .02], color:[0 0 1 ], contact:0, joint:rigid, logical: {goal} }
goal4 (floor){ shape:ssBox, Q:"t(-1 1 .1)", size:[.4 .4 .2 .02], color:[0 0 1], contact:0, joint:rigid, logical: {goal} }
goal5 (floor){ shape:ssBox, Q:"t(.5 -1 .1)", size:[.4 .4 .2 .02], color:[0 0 1], contact:0, joint:rigid, logical: {goal} }

##############################################################################################################################
obj1Joint(world){ Q:[1.6 1.6 0.1] }
obj1(obj1Joint) { type:ssBox size:[.25 .25 .2 .02] color:[1. 0. 0.],  logical={ movable_go }, joint:rigid, contact: 1 }
obj1_cam(obj1){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj2Joint(world){ Q:[.8 .8 0.1] }
obj2(obj2Joint) { type:ssBox size:[.25 .25 .2 .02] color:[1. 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj2_cam(obj2){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj3Joint(world){ Q:[-1.6 1.6 0.1] }
obj3(obj3Joint) { type:ssBox size:[.25 .25 .2 .02] color:[1. 0. 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj3_cam(obj3){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj4Joint(world){ Q:[.6 1.4 0.1] }
obj4(obj4Joint) { type:ssBox size:[.4 .4 .2 .02] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj4_cam(obj4){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obj5Joint(world){ Q:[1.4 .6 0.1] }
obj5(obj5Joint) { type:ssBox size:[.4 .4 .2 .02] color:[1 0 0],  logical={ movable_go }, joint:rigid, contact: 1 }
obj5_cam(obj5){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################
obs1Joint(world){ Q:[-1.4 -.6 0.1] }
obs1(obs1Joint) { type:ssBox size:[.4 .4 .2 .02] color:[1 0 0],  logical={ static }, joint:rigid, contact: 1 }
obs1_cam(obs1){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

obs2Joint(world){ Q:[-.6 -1.4 0.1] }
obs2(obs2Joint) { type:ssBox size:[.4 .4 .2 .02] color:[1 0 0],  logical={ static }, joint:rigid, contact: 1 }
obs2_cam(obs2){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}
