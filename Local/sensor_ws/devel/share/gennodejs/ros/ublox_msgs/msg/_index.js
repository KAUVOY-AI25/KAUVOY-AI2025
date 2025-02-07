
"use strict";

let MonHW = require('./MonHW.js');
let UpdSOS_Ack = require('./UpdSOS_Ack.js');
let CfgDAT = require('./CfgDAT.js');
let EsfINS = require('./EsfINS.js');
let CfgSBAS = require('./CfgSBAS.js');
let AidEPH = require('./AidEPH.js');
let NavSBAS = require('./NavSBAS.js');
let EsfSTATUS_Sens = require('./EsfSTATUS_Sens.js');
let NavATT = require('./NavATT.js');
let NavTIMEGPS = require('./NavTIMEGPS.js');
let NavCLOCK = require('./NavCLOCK.js');
let NavPVT = require('./NavPVT.js');
let RxmRAWX = require('./RxmRAWX.js');
let AidALM = require('./AidALM.js');
let NavPOSLLH = require('./NavPOSLLH.js');
let CfgHNR = require('./CfgHNR.js');
let CfgUSB = require('./CfgUSB.js');
let EsfRAW_Block = require('./EsfRAW_Block.js');
let NavVELECEF = require('./NavVELECEF.js');
let NavDGPS_SV = require('./NavDGPS_SV.js');
let MonVER = require('./MonVER.js');
let CfgPRT = require('./CfgPRT.js');
let AidHUI = require('./AidHUI.js');
let NavSVIN = require('./NavSVIN.js');
let NavSAT = require('./NavSAT.js');
let CfgGNSS_Block = require('./CfgGNSS_Block.js');
let RxmALM = require('./RxmALM.js');
let EsfMEAS = require('./EsfMEAS.js');
let CfgNMEA = require('./CfgNMEA.js');
let NavSTATUS = require('./NavSTATUS.js');
let CfgANT = require('./CfgANT.js');
let CfgINF_Block = require('./CfgINF_Block.js');
let RxmSVSI_SV = require('./RxmSVSI_SV.js');
let Ack = require('./Ack.js');
let RxmRAW_SV = require('./RxmRAW_SV.js');
let Inf = require('./Inf.js');
let NavSOL = require('./NavSOL.js');
let RxmSFRB = require('./RxmSFRB.js');
let EsfSTATUS = require('./EsfSTATUS.js');
let CfgRST = require('./CfgRST.js');
let NavSBAS_SV = require('./NavSBAS_SV.js');
let NavPVT7 = require('./NavPVT7.js');
let CfgCFG = require('./CfgCFG.js');
let NavDGPS = require('./NavDGPS.js');
let NavVELNED = require('./NavVELNED.js');
let RxmSFRBX = require('./RxmSFRBX.js');
let NavSAT_SV = require('./NavSAT_SV.js');
let RxmRTCM = require('./RxmRTCM.js');
let NavPOSECEF = require('./NavPOSECEF.js');
let MonHW6 = require('./MonHW6.js');
let NavSVINFO = require('./NavSVINFO.js');
let MonVER_Extension = require('./MonVER_Extension.js');
let CfgNAVX5 = require('./CfgNAVX5.js');
let EsfRAW = require('./EsfRAW.js');
let UpdSOS = require('./UpdSOS.js');
let RxmRAWX_Meas = require('./RxmRAWX_Meas.js');
let CfgDGNSS = require('./CfgDGNSS.js');
let NavDOP = require('./NavDOP.js');
let NavTIMEUTC = require('./NavTIMEUTC.js');
let MgaGAL = require('./MgaGAL.js');
let CfgINF = require('./CfgINF.js');
let MonGNSS = require('./MonGNSS.js');
let CfgGNSS = require('./CfgGNSS.js');
let HnrPVT = require('./HnrPVT.js');
let CfgMSG = require('./CfgMSG.js');
let RxmSVSI = require('./RxmSVSI.js');
let CfgNMEA7 = require('./CfgNMEA7.js');
let NavSVINFO_SV = require('./NavSVINFO_SV.js');
let CfgNMEA6 = require('./CfgNMEA6.js');
let CfgNAV5 = require('./CfgNAV5.js');
let RxmRAW = require('./RxmRAW.js');
let RxmEPH = require('./RxmEPH.js');
let CfgTMODE3 = require('./CfgTMODE3.js');
let TimTM2 = require('./TimTM2.js');
let CfgRATE = require('./CfgRATE.js');
let NavRELPOSNED = require('./NavRELPOSNED.js');

module.exports = {
  MonHW: MonHW,
  UpdSOS_Ack: UpdSOS_Ack,
  CfgDAT: CfgDAT,
  EsfINS: EsfINS,
  CfgSBAS: CfgSBAS,
  AidEPH: AidEPH,
  NavSBAS: NavSBAS,
  EsfSTATUS_Sens: EsfSTATUS_Sens,
  NavATT: NavATT,
  NavTIMEGPS: NavTIMEGPS,
  NavCLOCK: NavCLOCK,
  NavPVT: NavPVT,
  RxmRAWX: RxmRAWX,
  AidALM: AidALM,
  NavPOSLLH: NavPOSLLH,
  CfgHNR: CfgHNR,
  CfgUSB: CfgUSB,
  EsfRAW_Block: EsfRAW_Block,
  NavVELECEF: NavVELECEF,
  NavDGPS_SV: NavDGPS_SV,
  MonVER: MonVER,
  CfgPRT: CfgPRT,
  AidHUI: AidHUI,
  NavSVIN: NavSVIN,
  NavSAT: NavSAT,
  CfgGNSS_Block: CfgGNSS_Block,
  RxmALM: RxmALM,
  EsfMEAS: EsfMEAS,
  CfgNMEA: CfgNMEA,
  NavSTATUS: NavSTATUS,
  CfgANT: CfgANT,
  CfgINF_Block: CfgINF_Block,
  RxmSVSI_SV: RxmSVSI_SV,
  Ack: Ack,
  RxmRAW_SV: RxmRAW_SV,
  Inf: Inf,
  NavSOL: NavSOL,
  RxmSFRB: RxmSFRB,
  EsfSTATUS: EsfSTATUS,
  CfgRST: CfgRST,
  NavSBAS_SV: NavSBAS_SV,
  NavPVT7: NavPVT7,
  CfgCFG: CfgCFG,
  NavDGPS: NavDGPS,
  NavVELNED: NavVELNED,
  RxmSFRBX: RxmSFRBX,
  NavSAT_SV: NavSAT_SV,
  RxmRTCM: RxmRTCM,
  NavPOSECEF: NavPOSECEF,
  MonHW6: MonHW6,
  NavSVINFO: NavSVINFO,
  MonVER_Extension: MonVER_Extension,
  CfgNAVX5: CfgNAVX5,
  EsfRAW: EsfRAW,
  UpdSOS: UpdSOS,
  RxmRAWX_Meas: RxmRAWX_Meas,
  CfgDGNSS: CfgDGNSS,
  NavDOP: NavDOP,
  NavTIMEUTC: NavTIMEUTC,
  MgaGAL: MgaGAL,
  CfgINF: CfgINF,
  MonGNSS: MonGNSS,
  CfgGNSS: CfgGNSS,
  HnrPVT: HnrPVT,
  CfgMSG: CfgMSG,
  RxmSVSI: RxmSVSI,
  CfgNMEA7: CfgNMEA7,
  NavSVINFO_SV: NavSVINFO_SV,
  CfgNMEA6: CfgNMEA6,
  CfgNAV5: CfgNAV5,
  RxmRAW: RxmRAW,
  RxmEPH: RxmEPH,
  CfgTMODE3: CfgTMODE3,
  TimTM2: TimTM2,
  CfgRATE: CfgRATE,
  NavRELPOSNED: NavRELPOSNED,
};
