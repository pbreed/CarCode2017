
class RunProperties:public config_obj
{
public:
RunProperties():config_obj(appdata,"RunProp") {};
    config_int    CalTime{20,"CalTime"};
    config_bool   UseMag{true,"UseMag"};
	config_bool    StopOnRcLoss{true,"saferc"};
	config_double SteerZero{0.00,"SteerZero"};
    config_double ODODist{4.58,"OdoDist"};
	config_double CrossAngleScale{1,"XandScale"};
	config_double CrossMaxCorrect{45,"MaxCorrect"};
	config_double SteerP{0.02,"SteerP"};
	config_double SteerD{0,"SteerD"};
	config_double SteerI{0,"SteerI"};
	config_double SteerVE{0.002,"SteerVE"};
	config_double Brake{0.50,"Brake"};
	config_double DefSpeed{6.0,"DefSpeed"};
	config_double Speed_G{0.02,"SpeedG"};
	config_double Speed_M{0.015185185,"SpeedM"};
	config_double Speed_B{0.056666667,"SpeedB"};
	ConfigEndMarker;
};



extern RunProperties RunProps;
