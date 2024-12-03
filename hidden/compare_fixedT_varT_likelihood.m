[musV,omegasV,x_opV,y_opV]=connect_actions_with_smooth_trajectory_var_T([0 As(50) As(102) 0],[0 Os(252) Os(252) 0],sigma_ridge,speed_step,env,clearnce,Drift,Os,As,bestFitDataOffset,bestFitMeanToScaleRatio,...
bestNormalFitAngleScale,drift_fac);

[posteriorV]=Bayes_update_for_actions_params(1,musV,omegasV,sigma_ridge,As,Os,prior,wrkrs);



%%
[musF,omegasF,x_opF,y_opF]=connect_actions_with_smooth_trajectory([0 As(5) As(10) 0],[0 Os(252) Os(252) 0],sigma_ridge,speed_step,env,clearnce,Drift,50,Os,As,bestFitDataOffset,bestFitMeanToScaleRatio,...
bestNormalFitAngleScale,drift_fac);

[posteriorF]=Bayes_update_for_actions_params(1,musF,omegasF,sigma_ridge,As,Os,prior,wrkrs);
