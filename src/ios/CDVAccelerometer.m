/*
 Licensed to the Apache Software Foundation (ASF) under one
 or more contributor license agreements.  See the NOTICE file
 distributed with this work for additional information
 regarding copyright ownership.  The ASF licenses this file
 to you under the Apache License, Version 2.0 (the
 "License"); you may not use this file except in compliance
 with the License.  You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing,
 software distributed under the License is distributed on an
 "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 KIND, either express or implied.  See the License for the
 specific language governing permissions and limitations
 under the License.
 */

#import <CoreMotion/CoreMotion.h>
#import "CDVAccelerometer.h"

@interface CMDeviceMotion (TransformToReferenceFrame)
-(CMAcceleration)userAccelerationInReferenceFrame;
@end

@implementation CMDeviceMotion (TransformToReferenceFrame)

-(CMAcceleration)userAccelerationInReferenceFrame
{
    CMAcceleration acc = [self userAcceleration];
    CMRotationMatrix rot = [self attitude].rotationMatrix;

    CMAcceleration accRef;
    accRef.x = acc.x*rot.m11 + acc.y*rot.m12 + acc.z*rot.m13;
    accRef.y = acc.x*rot.m21 + acc.y*rot.m22 + acc.z*rot.m23;
    accRef.z = acc.x*rot.m31 + acc.y*rot.m32 + acc.z*rot.m33;

    return accRef;
}

@end

@interface CDVAccelerometer () {}
@property (readwrite, assign) BOOL isRunning;
@property (readwrite, assign) BOOL haveReturnedResult;
@property (readwrite, strong) CMMotionManager* motionManager;
@end

@implementation CDVAccelerometer

@synthesize callbackId, isRunning;

// defaults to 10 msec
#define kAccelerometerInterval 10
// g constant: -9.81 m/s^2
#define kGravitationalConstant -9.81

- (CDVAccelerometer*)init
{
    self = [super init];
    if (self) {
        self.callbackId = nil;
        self.isRunning = NO;
        self.haveReturnedResult = YES;
        self.motionManager = nil;
    }
    return self;
}

- (void)dealloc
{
    [self stop:nil];
}

- (void)start:(CDVInvokedUrlCommand*)command
{
    self.haveReturnedResult = NO;
    self.callbackId = command.callbackId;

    if (!self.motionManager)
    {
        self.motionManager = [[CMMotionManager alloc] init];
    }

    if (self.motionManager.deviceMotionAvailable) {
        // Assign the update interval to the motion manager and start updates
        self.motionManager.deviceMotionUpdateInterval = kAccelerometerInterval/1000;  // expected in seconds
        __weak CDVAccelerometer* weakSelf = self;
        [self.motionManager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXTrueNorthZVertical toQueue:[NSOperationQueue mainQueue] withHandler:^(CMDeviceMotion *motion, NSError *error) {
            if (motion)
                [weakSelf returnAccelInfo:motion];
        }];

        if (!self.isRunning) {
            self.isRunning = YES;
        }
    }
    else {

        NSLog(@"Running in Simulator? All gyro tests will fail.");
        CDVPluginResult* result = [CDVPluginResult resultWithStatus:CDVCommandStatus_INVALID_ACTION messageAsString:@"Error. Accelerometer Not Available."];

        [self.commandDelegate sendPluginResult:result callbackId:self.callbackId];
    }

}

- (void)onReset
{
    [self stop:nil];
}

- (void)stop:(CDVInvokedUrlCommand*)command
{
    if (self.motionManager.deviceMotionAvailable) {
        if (self.haveReturnedResult == NO){
            // block has not fired before stop was called, return whatever result we currently have
            //[self returnAccelInfo];
        }
        [self.motionManager stopDeviceMotionUpdates];
    }
    self.isRunning = NO;
}

- (void)returnAccelInfo:(CMDeviceMotion *)motion
{
    NSTimeInterval timestamp = ([[NSDate date] timeIntervalSince1970] * 1000);
    // Create an acceleration object
    NSMutableDictionary* accelProps = [NSMutableDictionary dictionaryWithCapacity:5];

    [accelProps setValue:[NSNumber numberWithDouble:(motion.userAcceleration.x + motion.gravity
.x) * kGravitationalConstant] forKey:@"x"];
    [accelProps setValue:[NSNumber numberWithDouble:(motion.userAcceleration.y + motion.gravity
.y) * kGravitationalConstant] forKey:@"y"];
    [accelProps setValue:[NSNumber numberWithDouble:(motion.userAcceleration.z + motion.gravity
.z) * kGravitationalConstant] forKey:@"z"];
    [accelProps setValue:[NSNumber numberWithInt:0] forKey:@"type"];
    [accelProps setValue:[NSNumber numberWithDouble:timestamp] forKey:@"timestamp"];

    CDVPluginResult* result = [CDVPluginResult resultWithStatus:CDVCommandStatus_OK messageAsDictionary:accelProps];
    [result setKeepCallback:[NSNumber numberWithBool:YES]];
    [self.commandDelegate sendPluginResult:result callbackId:self.callbackId];


    [accelProps setValue:[NSNumber numberWithDouble:motion.userAcceleration.x * kGravitationalConstant] forKey:@"x"];
    [accelProps setValue:[NSNumber numberWithDouble:motion.userAcceleration.y * kGravitationalConstant] forKey:@"y"];
    [accelProps setValue:[NSNumber numberWithDouble:motion.userAcceleration.z * kGravitationalConstant] forKey:@"z"];
    [accelProps setValue:[NSNumber numberWithInt:1] forKey:@"type"];

    result = [CDVPluginResult resultWithStatus:CDVCommandStatus_OK messageAsDictionary:accelProps];
    [result setKeepCallback:[NSNumber numberWithBool:YES]];
    [self.commandDelegate sendPluginResult:result callbackId:self.callbackId];


    CMAcceleration accel = [motion userAccelerationInReferenceFrame];

    [accelProps setValue:[NSNumber numberWithDouble:accel.x * kGravitationalConstant] forKey:@"x"];
    [accelProps setValue:[NSNumber numberWithDouble:accel.y * kGravitationalConstant] forKey:@"y"];
    [accelProps setValue:[NSNumber numberWithDouble:accel.z * kGravitationalConstant] forKey:@"z"];
    [accelProps setValue:[NSNumber numberWithInt:2] forKey:@"type"];

    result = [CDVPluginResult resultWithStatus:CDVCommandStatus_OK messageAsDictionary:accelProps];
    [result setKeepCallback:[NSNumber numberWithBool:YES]];
    [self.commandDelegate sendPluginResult:result callbackId:self.callbackId];
    self.haveReturnedResult = YES;
}

// TODO: Consider using filtering to isolate instantaneous data vs. gravity data -jm

/*
 #define kFilteringFactor 0.1

 // Use a basic low-pass filter to keep only the gravity component of each axis.
 grav_accelX = (acceleration.x * kFilteringFactor) + ( grav_accelX * (1.0 - kFilteringFactor));
 grav_accelY = (acceleration.y * kFilteringFactor) + ( grav_accelY * (1.0 - kFilteringFactor));
 grav_accelZ = (acceleration.z * kFilteringFactor) + ( grav_accelZ * (1.0 - kFilteringFactor));

 // Subtract the low-pass value from the current value to get a simplified high-pass filter
 instant_accelX = acceleration.x - ( (acceleration.x * kFilteringFactor) + (instant_accelX * (1.0 - kFilteringFactor)) );
 instant_accelY = acceleration.y - ( (acceleration.y * kFilteringFactor) + (instant_accelY * (1.0 - kFilteringFactor)) );
 instant_accelZ = acceleration.z - ( (acceleration.z * kFilteringFactor) + (instant_accelZ * (1.0 - kFilteringFactor)) );


 */
@end
