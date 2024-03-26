SonicMovePositionData
{
	var <position, <speed, <covarianceMat, <covarianceDiff;

	*new { |pos = #[0,0,0] , covariance =#[[0,0,0],[0,0,0],[0,0,0]], speedIn = #[0,0,0] , covarianceDiffIn =#[[0,0,0],[0,0,0],[0,0,0]] |
        ^super.new.init(pos, covariance, speedIn, covarianceDiffIn);
    }

	init { |pos, covariance, speedIn, covarianceDiffIn|
		position = pos;
		speed = speedIn;
		covarianceMat = covariance;
		covarianceDiff = covarianceDiffIn;
	}

	update{ | pos, covarMat |

		speed = pos - position;
		position = pos;

		covarianceDiff = covarMat - covarianceMat;
		covarianceMat = covarMat;

	}

}

SonicMoveXsensData
{
	//4*3
	// acceleration * 3, totAcc, trig, gyro * 3, totRotation, mag * 3, orientation * 3
	var <acceleration, <accelerationMagnitude, <jerk, <accTrig, <angularVelocity, <angularVelocityMagnitude, <magneticOrientation, <euler, <angleHistory, <quaternion, energy, accIntegration;
	var idx, <fftAngles, fftSize, cosTable;
	var <quadrant, <base, >scaleQuadrant;
	var busVals;
	var <buffer,idxBuf,<mfcc, <mfccResults, <arrMFCC;

	*new {
		|
		accelerationIn 				= #[0,0,0] ,
		accelerationMagnitudeIn		= 0.0,
		accTrigIn 	 				= 0.0 ,
		angularVelocityIn 			= #[0,0,0] ,
		angularVelocityMagnitudeIn 	= 0.0 ,
		magneticOrientationIn 		= #[0,0,0] ,
		eulerIn 					= #[0,0,0],
		quaternionIn 				= #[0,0,0,0],
		classificationBase			= 2,
		jerkIn         				= #[0,0,0], //not used for new usually
		energy						= 0.0,
		accIntegration				= 0.0


		|
        ^super.new.init(accelerationIn, jerkIn, accelerationMagnitudeIn, accTrigIn, angularVelocityIn, angularVelocityMagnitudeIn, magneticOrientationIn, eulerIn, quaternionIn, classificationBase);
    }

	init { |accelerationIn, jerkIn, accelerationMagnitudeIn, accTrigIn, angularVelocityIn, angularVelocityMagnitudeIn, magneticOrientationIn, eulerIn, quaternionIn, classificationBase|

		acceleration=accelerationIn;
		jerk = jerkIn;
		accelerationMagnitude = accelerationMagnitudeIn;
		accTrig = accTrigIn;
		angularVelocity = angularVelocityIn;
		angularVelocityMagnitude = angularVelocityMagnitudeIn;
		magneticOrientation = magneticOrientationIn;
		euler = eulerIn;
		quaternion=quaternionIn;
		quadrant = [0,0,0];
		base = classificationBase;
		scaleQuadrant = [5.0,10.0,10.0];
		idx=0;
		angleHistory = [0,0,0]!32;
		fftAngles = [0,0,0]!32;
		fftSize = 32;
		cosTable = Signal.fftCosTable(fftSize);
		energy=0.0;
		accIntegration = 0.0;
		// euler++jerk++acceleration++angularVelocity++quaternion++quadrant++energy
		busVals = Bus.control(Server.local, 18);

		arrMFCC= ((0!1024)!18);
		buffer = Buffer.alloc(Server.local, 1024,1);
		//mfcc = {Buffer(Server.local)}!18;
		//mfccResults= {Buffer.alloc(Server.local, 13, 1)}!16;
		idxBuf=0;
	}


	update{ | accelerationIn, accelerationMagnitudeIn, accTrigIn, angularVelocityIn, angularVelocityMagnitudeIn, magneticOrientationIn, eulerIn, quaternionIn, calcFFT = false |
		var tempQuad;
		var tempData;
		jerk = accelerationIn - acceleration;
		acceleration=accelerationIn;
		accelerationMagnitude = accelerationMagnitudeIn;
		accTrig = accTrigIn;
		angularVelocity = angularVelocityIn*2pi/360.0;
		angularVelocityMagnitude = angularVelocityMagnitudeIn;
		magneticOrientation = magneticOrientationIn;
		euler = eulerIn;
		quaternion=quaternionIn;

		tempQuad = ([jerk,acceleration, angularVelocity ]*scaleQuadrant) .clip(-1.0,1.0).collect({|it| it.maxItem({ arg item, i; abs(item) })});
		tempQuad=( base *(tempQuad.abs)).asInteger.clip(0,base-1);
		quadrant=( tempQuad[0]) + ((base.pow(1)) * tempQuad[1]) + ((base.pow(2)) * tempQuad[2]);

		accIntegration =  (acceleration.pow(2).reduce('+').sqrt) + ( 0.997 * accIntegration );

		energy = accIntegration + (angularVelocity.pow(2).reduce('+'));

		tempData = euler++ jerk++acceleration++angularVelocity++quaternion++quadrant++energy;
		busVals.setn( tempData );

		tempData.collect({|item,i| arrMFCC[i][idxBuf] = tempData[i]; });

		buffer.set(idxBuf, [acceleration++angularVelocity++quaternion].flat.reduce('+')/10.0 );

		//buffer.do({|buf,i| buf.set(idxBuf, tempData[i])  });



		//mfcc.collect({|buf,i|
		//FluidBufMFCC.processBlocking(Server.local, buffer[i], (idxBuf+1)&1023, startCoeff:1, features:buf);
		//});

		idxBuf = (idxBuf+1)&1023;




		if(calcFFT,{
						angleHistory[idx] = euler;
						idx=(idx+1)&0x1F;



						angleHistory.flop.do({
							|item, i|


							var real, imag, complex;

							real = Signal.newFrom(item);

							imag = Signal.newClear(fftSize); // zeros

							complex = fft(real, imag, cosTable);

							//(complex.magnitude* Signal.newFrom( [0]++(1!255)++(0!256) ))
							fftAngles[i] = (complex.magnitude / fftSize).asArray;





						});
					});






	}

	enumerateQuadrantStates
	{

		^ ((base.pow(3)))

	}
	getValueBus
	{

		^busVals
	}
	getBuf
	{

		^buffer
	}
}

SonicMoveXsensCorrelations
{
	var <correlationSelf,  <correlationOthers, <nDancers, <nSensors, combinationsSelf, combinationsOthers;
	var dataBusSelf, dataBusOthers;

	*new {
		|
		correlationSelfIn = #[],
		correlationOthersIn = #[],
		nDancersIn = 0,
		nSensorsIn = 0
		|
        ^super.new.init(correlationSelfIn, correlationOthersIn, nDancersIn, nSensorsIn);
    }

	init { |correlationSelfIn, correlationOthersIn, nDancersIn, nSensorsIn|

		correlationSelf=correlationSelfIn;
		correlationOthers = correlationOthersIn;
		nDancers = nDancersIn;
		nSensors = nSensorsIn;
		combinationsSelf = this.getCombinations(nSensorsIn);
		combinationsOthers = this.getCombinations(nDancersIn);
		dataBusSelf=nil;
		dataBusOthers=nil;

	}

	getCombinations
	{
	|n|
	^n.collect(_+1).powerset.select({|i|i.size==2}).sort({|a,b|  if(a[0]!=b[0],{ a[0]<b[0] },{a[1]<b[1]})   })
	}

	resetCombosSelf
	{

		combinationsSelf = this.getCombinations(nSensors);

	}


	resetCombosOthers
	{

		combinationsOthers = this.getCombinations(nDancers);

	}

	setNDancers{
	|nDancersIn|
		if(nDancers!=nDancersIn,{
			nDancers = nDancersIn;
			this.resetCombosOthers.value();

	});

	}
	setNSensors{
	|nSensorsIn|
		if(nSensors!=nSensorsIn,{
			nSensors = nSensorsIn;
			this.resetCombosSelf.value(); });


	}

	updateSelf {
		|data|
		//4*3
		// orientation * 3, acceleration * 3, gyro * 3, mag * 3,
		// nDancers * nSensors * 3 * 4
		if((nDancers!=0) && (nSensors!=0),{


			var paramCombos = nSensors.collect({|i|i}).sum;
			var expectedSize = paramCombos * 4 * 3 * nDancers;
			//ORI ACC GYR MAG * x y z = 4*3
			if(data.size == expectedSize,{

				correlationSelf = data.clump(3).clump(4).clump(paramCombos);   //.clump(3).clump(4).clump(paramCombos);//.clump(nDancers);
				//indexes =
			});
		});
		dataBusSelf ?? {dataBusSelf = Bus.control(Server.local, data.flat.size); };
		if((dataBusSelf.numChannels==data.flat.size)&&(dataBusSelf!=nil),{dataBusSelf.setn(data.flat)});

	}
	updateOthers {
		|data|
		// nDancers * nSensors * 3 * 4
		if((nDancers!=0) && (nSensors!=0),{


			var dancerCombos = nDancers.collect({|i|i}).sum;
			var expectedSize = dancerCombos * 4 * 3 * nSensors;
			//ORI ACC GYR MAG * x y z = 4*3
			if(data.size == expectedSize,{

				correlationOthers = data.clump(3).clump(4).clump(dancerCombos);//.clump(paramCombos);

			});
		});
		dataBusOthers ?? {dataBusOthers= Bus.control(Server.local, data.flat.size);};
		if((dataBusOthers.numChannels==data.flat.size)&&(dataBusOthers!=nil),{dataBusOthers.setn(data.flat)});


	}

	getDataSelf{
		|nDancer, combo = #[1,1]|
		var idx = combinationsSelf.indexOfEqual(combo);
		^correlationSelf[nDancer][idx];

	}
	getDataOthers{
		|nSensor, combo = #[1,1]|
		var idx = combinationsOthers.indexOfEqual(combo);
		^correlationOthers[nSensor][idx];

	}
	getBusSelf
	{
		^dataBusSelf;
	}
	getBusOthers
	{
		^dataBusOthers;
	}

	getDataSetSelf{


		^correlationSelf;

	}
	getDataSetOthers{


		^correlationOthers;

	}

	getCombosSelf
	{
		^combinationsSelf;


	}
	getCombosOthers
	{
		^combinationsOthers;


	}



}

SonicMoveXsensFFT
{
	var >data, >stats, statsBus;

		*new {
		|
		dataIn = #[]
		statsIn = #[]
		|
        ^super.new.init(dataIn, statsIn);
    }

	init { |dataIn, statsIn|

		data  = dataIn;
		stats = statsIn;
		statsBus = nil;

	}

	updateBus
	{
		statsBus ?? {statsBus= Bus.control(Server.local, stats.flat.size);};
		if((statsBus.numChannels==stats.flat.size)&&(statsBus!=nil),{statsBus.setn(stats.flat)},
			{
				statsBus.free;
				statsBus= Bus.control(Server.local, stats.flat.size);
				statsBus.setn(stats.flat);


		});

	}
	getBus
	{
		^statsBus;
	}


	getFFTDancerAllSensors{
		|nDancer|
		var nDancerIdx = (nDancer-1).clip(0, data.size-1);


		^data[nDancerIdx];
	}
	getFFTStatsDancerAllSensors{
		|nDancer|
		var nDancerIdx = (nDancer-1).clip(0, data.size-1);


		^stats[nDancerIdx];
	}

	getFFTSensor{
		|nDancer, nSensor|
		var nDancerIdx = (nDancer-1).clip(0, data.size-1);
		var nSensorIdx = (nSensor-1).clip(0, data[nDancerIdx].size-1);

		^data[nDancerIdx][nSensorIdx];
	}

	getFFTStatsSensor
	{
		|nDancer, nSensor|
		var nDancerIdx = (nDancer-1).clip(0, data.size-1);
		var nSensorIdx = (nSensor-1).clip(0, data[nDancerIdx].size-1);

		^stats[nDancerIdx][nSensorIdx];

	}


}
