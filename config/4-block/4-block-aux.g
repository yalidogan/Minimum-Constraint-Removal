Include: <../base-walls-min-heatmap.g>
cam0(world){ Q:"t(0 0 10) d(180 1 0 0)" shape:camera width:300 height:300}
#########################################################################################################################
wall2_h (world){ shape:ssBox, Q:"t(1 -.75 0.3)", size:[.1 2.5 0.6 .02], color:[1 0 0],contact: 1 }
#########################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(1.6 -0.5 .2)", size:[.3 .3 0.2 .02], color:[0 0 1], contact:0, logical:{goal} }
#########################################################################################################################
egoJoint(world){Q:[0 0 .2]  }
ego(egoJoint) {
    shape:ssCylinder, Q:[-1.3 -1.3, 0], size:[.4 .02 .02], color:[1 1 0]
}
#########################################################################################################################
obj1Joint(world){ Q:[0.0 0.0 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, Q:[-1.5 1.5 .0], size:[.3 .3 0.2 .02], logical={ object }  nomass:1, color:[0 0 1],
    joint:rigid, friction:.1  contact: 1
}
obj1_cam(obj1){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}
#########################################################################################################################
obs1Joint(world){ Q:[0.0 0.0 0.1] } # works
obs1(obs1Joint) {
    shape:ssBox, Q:[-1.2 -0.3 .0], size:[1 0.2 .2 .02], logical={ object } nomass:1,  color:[1 0 0],
    joint:rigid, friction:.1  contact: 1
}
obs1_cam(obs1){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}
#########################################################################################################################
obs2Joint(world){ Q:[0.0 0.0 0.1] } # works
obs2(obs2Joint) {
    shape:ssBox, Q:[-0.75 -1.2 .0], size:[.2 1 .2 .02], logical={ object } nomass:1,  color:[1 0 0],
    joint:rigid, friction:.1  contact: 1
}
obs2_cam(obs2){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}
#########################################################################################################################
obs3Joint(world){ Q:[0.0 0.0 0.1] } # works
obs3(obs3Joint) {
    shape:ssBox, Q:[-0.5 -1.3 .0], size:[.2 1 .2 .02], logical={ object } nomass:1,  color:[1 0 0],
    joint:rigid, friction:.1  contact: 1
}
obs3_cam(obs3){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}
#########################################################################################################################
obs4Joint(world){ Q:[0.0 0.0 0.1] } # works
obs4(obs4Joint) {
    shape:ssBox, Q:[-1.4 -0.75 .0], size:[1 0.2 .2 .02], logical={ object } nomass:1,  color:[1 0 0],
    joint:rigid, friction:.1  contact: 1
}
obs4_cam(obs4){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}


