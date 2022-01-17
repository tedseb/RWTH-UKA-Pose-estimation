(self["webpackChunkshowroom"] = self["webpackChunkshowroom"] || []).push([["main"],{

/***/ 8255:
/*!*******************************************************!*\
  !*** ./$_lazy_route_resources/ lazy namespace object ***!
  \*******************************************************/
/***/ ((module) => {

function webpackEmptyAsyncContext(req) {
	// Here Promise.resolve().then() is used instead of new Promise() to prevent
	// uncaught exception popping up in devtools
	return Promise.resolve().then(() => {
		var e = new Error("Cannot find module '" + req + "'");
		e.code = 'MODULE_NOT_FOUND';
		throw e;
	});
}
webpackEmptyAsyncContext.keys = () => ([]);
webpackEmptyAsyncContext.resolve = webpackEmptyAsyncContext;
webpackEmptyAsyncContext.id = 8255;
module.exports = webpackEmptyAsyncContext;

/***/ }),

/***/ 158:
/*!***************************************!*\
  !*** ./src/app/app-routing.module.ts ***!
  \***************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "AppRoutingModule": () => (/* binding */ AppRoutingModule)
/* harmony export */ });
/* harmony import */ var _angular_router__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/router */ 1258);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ 2316);



const routes = [];
class AppRoutingModule {
}
AppRoutingModule.ɵfac = function AppRoutingModule_Factory(t) { return new (t || AppRoutingModule)(); };
AppRoutingModule.ɵmod = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineNgModule"]({ type: AppRoutingModule });
AppRoutingModule.ɵinj = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineInjector"]({ imports: [[_angular_router__WEBPACK_IMPORTED_MODULE_1__.RouterModule.forRoot(routes)], _angular_router__WEBPACK_IMPORTED_MODULE_1__.RouterModule] });
(function () { (typeof ngJitMode === "undefined" || ngJitMode) && _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵsetNgModuleScope"](AppRoutingModule, { imports: [_angular_router__WEBPACK_IMPORTED_MODULE_1__.RouterModule], exports: [_angular_router__WEBPACK_IMPORTED_MODULE_1__.RouterModule] }); })();


/***/ }),

/***/ 5041:
/*!**********************************!*\
  !*** ./src/app/app.component.ts ***!
  \**********************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "AppComponent": () => (/* binding */ AppComponent)
/* harmony export */ });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _step_step_component__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./step/step.component */ 6502);


class AppComponent {
    constructor() {
        this.title = 'Showroom Application';
    }
}
AppComponent.ɵfac = function AppComponent_Factory(t) { return new (t || AppComponent)(); };
AppComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵdefineComponent"]({ type: AppComponent, selectors: [["app-root"]], decls: 1, vars: 0, template: function AppComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelement"](0, "app-step");
    } }, directives: [_step_step_component__WEBPACK_IMPORTED_MODULE_0__.StepComponent], styles: ["\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IiIsImZpbGUiOiJhcHAuY29tcG9uZW50LnNjc3MifQ== */"] });


/***/ }),

/***/ 6747:
/*!*******************************!*\
  !*** ./src/app/app.module.ts ***!
  \*******************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "AppModule": () => (/* binding */ AppModule)
/* harmony export */ });
/* harmony import */ var _angular_platform_browser__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! @angular/platform-browser */ 1570);
/* harmony import */ var _app_routing_module__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./app-routing.module */ 158);
/* harmony import */ var _app_component__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./app.component */ 5041);
/* harmony import */ var _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(/*! @angular/platform-browser/animations */ 718);
/* harmony import */ var _step_step_component__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./step/step.component */ 6502);
/* harmony import */ var _step_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./step/renderer/renderer.component */ 2979);
/* harmony import */ var _step_intro_intro_component__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./step/intro/intro.component */ 2338);
/* harmony import */ var _angular_common_http__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(/*! @angular/common/http */ 3882);
/* harmony import */ var _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(/*! @angular/material/grid-list */ 5937);
/* harmony import */ var _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(/*! @angular/material/snack-bar */ 8456);
/* harmony import */ var _angular_material_tabs__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(/*! @angular/material/tabs */ 9348);
/* harmony import */ var _angular_material_core__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(/*! @angular/material/core */ 2220);
/* harmony import */ var _videogular_ngx_videogular_core__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(/*! @videogular/ngx-videogular/core */ 1101);
/* harmony import */ var _videogular_ngx_videogular_controls__WEBPACK_IMPORTED_MODULE_15__ = __webpack_require__(/*! @videogular/ngx-videogular/controls */ 6421);
/* harmony import */ var _videogular_ngx_videogular_overlay_play__WEBPACK_IMPORTED_MODULE_16__ = __webpack_require__(/*! @videogular/ngx-videogular/overlay-play */ 2938);
/* harmony import */ var _videogular_ngx_videogular_buffering__WEBPACK_IMPORTED_MODULE_17__ = __webpack_require__(/*! @videogular/ngx-videogular/buffering */ 1167);
/* harmony import */ var _step_select_select_component__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ./step/select/select.component */ 3710);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! @angular/core */ 2316);


















class AppModule {
}
AppModule.ɵfac = function AppModule_Factory(t) { return new (t || AppModule)(); };
AppModule.ɵmod = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_6__["ɵɵdefineNgModule"]({ type: AppModule, bootstrap: [_app_component__WEBPACK_IMPORTED_MODULE_1__.AppComponent] });
AppModule.ɵinj = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_6__["ɵɵdefineInjector"]({ providers: [], imports: [[
            _angular_platform_browser__WEBPACK_IMPORTED_MODULE_7__.BrowserModule,
            _app_routing_module__WEBPACK_IMPORTED_MODULE_0__.AppRoutingModule,
            _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_8__.BrowserAnimationsModule,
            _angular_common_http__WEBPACK_IMPORTED_MODULE_9__.HttpClientModule,
            _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_10__.MatGridListModule,
            _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_11__.MatSnackBarModule,
            _angular_material_tabs__WEBPACK_IMPORTED_MODULE_12__.MatTabsModule,
            _angular_material_core__WEBPACK_IMPORTED_MODULE_13__.MatCommonModule,
            _videogular_ngx_videogular_core__WEBPACK_IMPORTED_MODULE_14__.VgCoreModule,
            _videogular_ngx_videogular_controls__WEBPACK_IMPORTED_MODULE_15__.VgControlsModule,
            _videogular_ngx_videogular_overlay_play__WEBPACK_IMPORTED_MODULE_16__.VgOverlayPlayModule,
            _videogular_ngx_videogular_buffering__WEBPACK_IMPORTED_MODULE_17__.VgBufferingModule
        ]] });
(function () { (typeof ngJitMode === "undefined" || ngJitMode) && _angular_core__WEBPACK_IMPORTED_MODULE_6__["ɵɵsetNgModuleScope"](AppModule, { declarations: [_app_component__WEBPACK_IMPORTED_MODULE_1__.AppComponent,
        _step_step_component__WEBPACK_IMPORTED_MODULE_2__.StepComponent,
        _step_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_3__.RendererComponent,
        _step_intro_intro_component__WEBPACK_IMPORTED_MODULE_4__.IntroComponent,
        _step_select_select_component__WEBPACK_IMPORTED_MODULE_5__.SelectComponent], imports: [_angular_platform_browser__WEBPACK_IMPORTED_MODULE_7__.BrowserModule,
        _app_routing_module__WEBPACK_IMPORTED_MODULE_0__.AppRoutingModule,
        _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_8__.BrowserAnimationsModule,
        _angular_common_http__WEBPACK_IMPORTED_MODULE_9__.HttpClientModule,
        _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_10__.MatGridListModule,
        _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_11__.MatSnackBarModule,
        _angular_material_tabs__WEBPACK_IMPORTED_MODULE_12__.MatTabsModule,
        _angular_material_core__WEBPACK_IMPORTED_MODULE_13__.MatCommonModule,
        _videogular_ngx_videogular_core__WEBPACK_IMPORTED_MODULE_14__.VgCoreModule,
        _videogular_ngx_videogular_controls__WEBPACK_IMPORTED_MODULE_15__.VgControlsModule,
        _videogular_ngx_videogular_overlay_play__WEBPACK_IMPORTED_MODULE_16__.VgOverlayPlayModule,
        _videogular_ngx_videogular_buffering__WEBPACK_IMPORTED_MODULE_17__.VgBufferingModule] }); })();


/***/ }),

/***/ 2586:
/*!************************************************!*\
  !*** ./src/app/services/coordinate.service.ts ***!
  \************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "CoordinateService": () => (/* binding */ CoordinateService)
/* harmony export */ });
/* harmony import */ var rxjs__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! rxjs */ 6491);
/* harmony import */ var rxjs__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! rxjs */ 3413);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _data_service__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./data.service */ 2468);



class CoordinateService {
    constructor(dataService) {
        this.dataService = dataService;
        this.matrabs_used = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23];
        this.matrabs_labels = ['M_Hip', 'L_Hip', 'R_Hip', 'L_Back', 'L_Knee', 'R_Knee', 'M_Back', 'L_Ankle', 'R_Ankle', 'U_Back', 'L_Toes', 'R_Toes', 'Neck', 'L_Collarbone', 'R_Collarbone', 'Head', 'L_Shoulder', 'R_Shoulder', 'L_Elbow', 'R_Elbow', 'L_Wrist', 'R_Wrist', 'L_Fingers', 'R_Fingers'];
        this.matrabs = [[1, 4], [1, 0], [2, 5], [2, 0], [3, 6], [3, 0], [4, 7], [5, 8], [6, 9], [7, 10], [8, 11], [9, 12], [12, 13], [12, 14], [12, 15], [13, 16], [14, 17], [16, 18], [17, 19], [18, 20], [19, 21], [20, 22], [21, 23]];
        this.connections = {};
        this.socket = dataService.getWebsocket();
        this.socket.subscribe(message => {
            if (message.usage == null && message.topic == null) {
                this.lastPose = message;
                this.update.next(this.lastPose);
            }
            else if (message.usage && message.usage === "reference_progress") {
                this.progression.next(message.data.data);
            }
            else if (message.usage && message.usage === "reference_frame") {
                this.frame.next(message.data.data);
            }
        });
        this.update = new rxjs__WEBPACK_IMPORTED_MODULE_1__.BehaviorSubject(this.lastPose);
        this.progression = new rxjs__WEBPACK_IMPORTED_MODULE_2__.ReplaySubject();
        this.frame = new rxjs__WEBPACK_IMPORTED_MODULE_2__.ReplaySubject();
        /* this.connections = {
          OP_R_Shoulder: ['OP_R_Elbow', 'R_Hip'],
          OP_R_Elbow: ['OP_R_Wrist'],
          OP_L_Shoulder: ['OP_L_Elbow', 'L_Hip', 'OP_R_Shoulder'],
          OP_L_Elbow: ['OP_L_Wrist'],
          R_Hip: ['OP_R_Hip', 'L_Hip', 'OP_R_Knee'],
          OP_R_Hip: ['OP_L_Hip', 'OP_R_Knee'],
          OP_R_Knee: ['OP_R_Ankle'],
          OP_L_Hip: ['OP_L_Knee'],
          L_Hip: ['OP_L_Hip', 'OP_L_Knee'],
          OP_L_Knee: ['OP_L_Ankle'],
          OP_L_Ankle: ['OP_L_Heel'],
          OP_L_Heel: ['OP_L_Small_Toe', 'OP_L_Big_Toe'],
          OP_L_Small_Toe: ['OP_L_Big_Toe'],
          OP_R_Ankle: ['OP_R_Heel'],
          OP_R_Heel: ['OP_R_Big_Toe'],
          OP_R_Big_Toe: ['OP_R_Small_Toe'],
          OP_R_Small_Toe: ['OP_R_Heel'],
          Jaw_HM: ['OP_R_Ear', 'OP_L_Ear', 'OP_Nose'],
          OP_Nose: ['OP_R_Eye', 'OP_L_Eye'],
          OP_R_Eye: ['OP_L_Eye'],
          OP_R_Ear: ['Head_HM'],
          OP_L_Ear: ['Head_HM'],
          OP_Neck: ['Neck_LSP'],
          Neck_LSP: ['Head_HM'],
          Spine_HM: ['Neck_LSP', 'Pelvis_MPII']
        }; */
        this.dataService.getConnectionsDict().subscribe((val) => this.buildConnectionsDict(val));
        //this.useMocking();
    }
    buildConnectionsDict(val) {
        let dict = {};
        const used = val.used;
        const labels = val.labels;
        const skeleton = val.connections;
        skeleton.forEach(val => {
            if (dict[labels[val[0]]]) {
                dict[labels[val[0]]].push(labels[val[1]]);
            }
            else {
                dict[labels[val[0]]] = [labels[val[1]]];
            }
        });
        console.log(dict);
        this.connections = dict;
    }
}
CoordinateService.ɵfac = function CoordinateService_Factory(t) { return new (t || CoordinateService)(_angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵinject"](_data_service__WEBPACK_IMPORTED_MODULE_0__.DataService)); };
CoordinateService.ɵprov = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵdefineInjectable"]({ token: CoordinateService, factory: CoordinateService.ɵfac, providedIn: 'root' });


/***/ }),

/***/ 2468:
/*!******************************************!*\
  !*** ./src/app/services/data.service.ts ***!
  \******************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "DataService": () => (/* binding */ DataService)
/* harmony export */ });
/* harmony import */ var rxjs_webSocket__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! rxjs/webSocket */ 7717);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _angular_common_http__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/common/http */ 3882);



class DataService {
    constructor(http) {
        this.http = http;
    }
    getWebsocket() {
        return (0,rxjs_webSocket__WEBPACK_IMPORTED_MODULE_0__.webSocket)('ws://localhost:6161');
    }
    disconnectWs() {
        // socket.complete();
    }
    sendWs(msg) {
        // socket.next(msg);
    }
    getLanguage() {
        this.http.get('api/language').subscribe((val) => {
            return val;
        });
    }
    getCoordinates() {
        this.http.get('api/coordinates').subscribe(val => {
            console.log(val);
        });
    }
    postPose(pose) {
        this.http.post('/api/pose', pose);
    }
    getConnections() {
        let connections;
        this.http.get('/api/connections').subscribe(val => {
            return val;
        });
        // return connections;
    }
    startRecording() {
        this.http.post('/api/exercise/recording/start', null).subscribe(val => {
        });
    }
    stopRecording() {
        return this.http.post('/api/exercise/recording/stop', null);
    }
    saveRecording(name, description, features, recording) {
        const exercise = {
            name,
            description,
            features,
            recording
        };
        this.http.post('/api/expert/recording/save', exercise).subscribe(val => { });
    }
    getConnectionsDict() {
        return this.http.get('/api/connections/dict');
    }
    // tslint:disable-next-line:typedef
    saveExerciseWithStages(stages, name, recording) {
        const toSave = {
            stages,
            recording,
            name
        };
        console.log(toSave);
        this.http.post('/api/expert/exercises/stages/save', toSave).subscribe(val => {
            console.log(val);
        });
    }
}
DataService.ɵfac = function DataService_Factory(t) { return new (t || DataService)(_angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵinject"](_angular_common_http__WEBPACK_IMPORTED_MODULE_2__.HttpClient)); };
DataService.ɵprov = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵdefineInjectable"]({ token: DataService, factory: DataService.ɵfac, providedIn: 'root' });


/***/ }),

/***/ 1359:
/*!********************************************!*\
  !*** ./src/app/services/speech.service.ts ***!
  \********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "SpeechService": () => (/* binding */ SpeechService)
/* harmony export */ });
/* harmony import */ var rxjs__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! rxjs */ 3413);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/core */ 2316);


class SpeechService {
    constructor() {
        // subscribable events
        this.welcome = new rxjs__WEBPACK_IMPORTED_MODULE_0__.ReplaySubject();
        this.begin = new rxjs__WEBPACK_IMPORTED_MODULE_0__.ReplaySubject();
        this.exercise = new rxjs__WEBPACK_IMPORTED_MODULE_0__.ReplaySubject();
        this.feedback = new rxjs__WEBPACK_IMPORTED_MODULE_0__.ReplaySubject();
        this.result = new rxjs__WEBPACK_IMPORTED_MODULE_0__.ReplaySubject();
        // intervals
        this.resultInterval = setInterval(() => {
            if (!this.processed) {
                if (Date.now() - this.timestamp >= 5000) {
                    this.wordConcat();
                    console.log(this.text);
                    this.processed = true;
                }
            }
        }, 7500);
        this.lang = 'de-DE';
        this.recognition = new webkitSpeechRecognition();
        this.isListening = false;
        this.processed = true;
        this.timestamp = Date.now();
        this.text = '';
        this.tempWords = '';
        this.dict = {
            'en-US': { welcome: 'welcome', begin: 'begin', exercise: ['push up', 'military press', 'dead lift'] },
            'de-DE': { welcome: 'willkommen', begin: 'begin', exercise: ['liegestütze', 'liege stütze', 'militär presse', 'kreuz heben', 'kreuzheben'] }
        };
        this.reason = false;
        this.init();
    }
    ngOnDestroy() {
        if (this.resultInterval) {
            clearInterval(this.resultInterval);
        }
    }
    ngOnInit() {
    }
    init() {
        this.recognition.interimResults = true;
        this.recognition.lang = this.lang;
        this.recognition.continuous = true;
        this.recognition.addEventListener('result', (speechEvent) => {
            this.processed = false;
            this.timestamp = Date.now();
            console.log(speechEvent);
            const transcript = Array.from(speechEvent.results)
                .map((result) => result[0])
                .map((result) => {
                console.log(result.transcript);
                return result.transcript;
            }).join('');
            const lt = transcript.toLowerCase();
            if (lt.includes("ende") && lt.includes("sprachsteuerung")) {
                console.log('BEENDE');
                this.reason = true;
                this.stop();
            }
            if (lt.includes(this.dict[this.lang].welcome) || (lt.includes('will') && lt.includes('kommen'))) {
                this.welcome.next(transcript);
            }
            if (lt.includes(this.dict[this.lang].begin)) {
                this.begin.next(transcript);
            }
            if (this.dict[this.lang].exercise.some(val => { lt.includes(val); })) {
                console.log("exercise known");
                this.exercise.next(transcript);
            }
            else if (lt.includes('übung') || (lt.includes('übe'))) {
                this.exercise.next(transcript);
            }
            else {
                this.result.next(lt);
            }
            this.tempWords = transcript;
        });
        this.recognition.addEventListener('end', () => {
            if (this.reason) {
                console.log('End speech recognition');
            }
            else {
                console.log('restarting speech recognition');
                this.start();
            }
        });
    }
    start() {
        this.isListening = true;
        this.recognition.start();
        console.log('Speech recognition started');
    }
    stop() {
        this.isListening = false;
        this.wordConcat();
        this.recognition.stop();
    }
    wordConcat() {
        this.text = this.text + ' ' + this.tempWords + '.';
        this.tempWords = '';
    }
}
SpeechService.ɵfac = function SpeechService_Factory(t) { return new (t || SpeechService)(); };
SpeechService.ɵprov = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵdefineInjectable"]({ token: SpeechService, factory: SpeechService.ɵfac, providedIn: 'root' });


/***/ }),

/***/ 2338:
/*!***********************************************!*\
  !*** ./src/app/step/intro/intro.component.ts ***!
  \***********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "IntroComponent": () => (/* binding */ IntroComponent)
/* harmony export */ });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ 2316);

class IntroComponent {
    constructor() { }
    ngOnInit() {
    }
}
IntroComponent.ɵfac = function IntroComponent_Factory(t) { return new (t || IntroComponent)(); };
IntroComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineComponent"]({ type: IntroComponent, selectors: [["app-intro"]], decls: 4, vars: 0, consts: [[1, "container"], [1, "header"]], template: function IntroComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "div", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](1, "div", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](2, "h1");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](3, "Welcome to the future.");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    } }, styles: [".header[_ngcontent-%COMP%] {\n  margin: 0;\n  top: 50%;\n  left: 50%;\n  position: absolute;\n  transform: translate(-50%, -50%);\n}\n\n.container[_ngcontent-%COMP%] {\n  position: fixed;\n  padding: 0;\n  margin: 0;\n  top: 0;\n  left: 0;\n  width: 100%;\n  height: 100%;\n  background: lightgray;\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImludHJvLmNvbXBvbmVudC5zY3NzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBO0VBQ0UsU0FBQTtFQUNBLFFBQUE7RUFDQSxTQUFBO0VBQ0Esa0JBQUE7RUFFQSxnQ0FBQTtBQUNGOztBQUVBO0VBQ0UsZUFBQTtFQUNBLFVBQUE7RUFDQSxTQUFBO0VBQ0EsTUFBQTtFQUNBLE9BQUE7RUFDQSxXQUFBO0VBQ0EsWUFBQTtFQUNBLHFCQUFBO0FBQ0YiLCJmaWxlIjoiaW50cm8uY29tcG9uZW50LnNjc3MiLCJzb3VyY2VzQ29udGVudCI6WyIuaGVhZGVyIHtcbiAgbWFyZ2luOiAwO1xuICB0b3A6IDUwJTtcbiAgbGVmdDogNTAlO1xuICBwb3NpdGlvbjogYWJzb2x1dGU7XG4gIC1tcy10cmFuc2Zvcm06IHRyYW5zbGF0ZSgtNTAlLCAtNTAlKTtcbiAgdHJhbnNmb3JtOiB0cmFuc2xhdGUoLTUwJSwgLTUwJSk7XG59XG5cbi5jb250YWluZXIge1xuICBwb3NpdGlvbjogZml4ZWQ7XG4gIHBhZGRpbmc6IDA7XG4gIG1hcmdpbjogMDtcbiAgdG9wOiAwO1xuICBsZWZ0OiAwO1xuICB3aWR0aDogMTAwJTtcbiAgaGVpZ2h0OiAxMDAlO1xuICBiYWNrZ3JvdW5kOnJnYigyMTEsMjExLDIxMSlcbn1cbiJdfQ== */"] });


/***/ }),

/***/ 2979:
/*!*****************************************************!*\
  !*** ./src/app/step/renderer/renderer.component.ts ***!
  \*****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "RendererComponent": () => (/* binding */ RendererComponent)
/* harmony export */ });
/* harmony import */ var three__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! three */ 4986);
/* harmony import */ var three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! three/examples/jsm/controls/OrbitControls.js */ 6668);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _services_data_service__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../../services/data.service */ 2468);
/* harmony import */ var _services_coordinate_service__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ../../services/coordinate.service */ 2586);
/* harmony import */ var _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! @angular/material/snack-bar */ 8456);






const _c0 = ["editorContainer"];
class RendererComponent {
    constructor(dataService, coordinateService, _snackbar) {
        this.dataService = dataService;
        this.coordinateService = coordinateService;
        this._snackbar = _snackbar;
        this.currentExercise = "";
        this.raycaster = new three__WEBPACK_IMPORTED_MODULE_3__.Raycaster();
        // properties
        this.viewHeight = window.innerHeight - 70 - 48 - 48;
        this.viewWidth = window.innerWidth;
        this.pixelRatio = window.devicePixelRatio;
        this.mouse = new three__WEBPACK_IMPORTED_MODULE_3__.Vector2();
        this.touch = new three__WEBPACK_IMPORTED_MODULE_3__.Vector2();
        // skeleton
        this.dots = [];
        this.dotsMapping = [];
        this.lines = [];
        this.max = 100;
        this.selectedJoint = '';
        this.color = [0xf72585, 0x4361ee, 0x4cc9f0, 0xb5179e, 0x4895ef, 0x7209b7];
        this.isInitialized = false;
        // video properties
        this.frame = 0;
        this.progression = 0;
        const width = 400;
        const height = 400;
        this.renderer = new three__WEBPACK_IMPORTED_MODULE_3__.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setClearColor(0xffffff, 0);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.scene = new three__WEBPACK_IMPORTED_MODULE_3__.Scene();
        this.scene.background = null;
        this.camera = new three__WEBPACK_IMPORTED_MODULE_3__.PerspectiveCamera(25, width / height, 0.1, 2000);
        this.orbitControls = new three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_0__.OrbitControls(this.camera, this.renderer.domElement);
        this.orbitControls.update();
    }
    // event listeners
    onWindowResize(event) {
        this.viewHeight = window.innerHeight - 70 - 48 - 48;
        const width = window.innerWidth;
        const height = window.innerHeight - 64;
        //this.renderer.setSize(width * 2 / 3, height);
        //this.camera.aspect = (width * 2 / 3) / height;
        this.camera.updateProjectionMatrix();
    }
    onDocumentMouseMove(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / (rect.right - rect.left)) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / (rect.bottom - rect.top)) * 2 + 1;
    }
    onTouchEnd(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.touch.x = ((event.changedTouches[0].clientX - rect.left) / (rect.right - rect.left)) * 2 - 1;
        this.touch.y = -((event.changedTouches[0].clientY - rect.top) / (rect.bottom - rect.top)) * 2 + 1;
    }
    ngOnInit() {
        this.initScene();
    }
    ngAfterViewInit() {
        this.camera.lookAt(this.scene.position);
        this.editorContainer.nativeElement.appendChild(this.renderer.domElement);
        let context = this.renderer.domElement.getContext('2d');
        this.renderer.domElement.setAttribute('style', 'background: transparent;');
        context === null || context === void 0 ? void 0 : context.clearRect(0, 0, this.renderer.domElement.width, this.renderer.domElement.height);
        this.renderer.autoClear = false;
        //this.editorContainer.nativeElement.addEventListener('click', (event: MouseEvent) => { this.addJointToStage(this.selectedJoint); });
        this.renderer.setPixelRatio(this.pixelRatio);
        this.renderer.setSize(400, 400);
        this.animate();
        this.coordinateService.update.subscribe(pose => {
            if (!this.isInitialized) {
                this.initSkeleton();
                return;
            }
            else if (pose) {
                this.recordingPose = pose;
                this.renderSkeleton(1, 0, 0, 0);
            }
        });
        this.coordinateService.frame.subscribe(frame => {
            this.frame = frame;
        });
        this.coordinateService.progression.subscribe(progression => {
            this.progression = progression;
            this.colorLines();
        });
    }
    ngOnDestroy() {
    }
    animate() {
        window.requestAnimationFrame(() => this.animate());
        this.renderer.clear();
        this.renderer.render(this.scene, this.camera);
        this.renderer.clearDepth();
        this.selectJoint();
    }
    initScene() {
        this.camera.position.set(20, 20, 20);
        this.orbitControls.enablePan = true;
        this.orbitControls.enableZoom = true;
        this.orbitControls.enableKeys = true;
        const axesHelper = new three__WEBPACK_IMPORTED_MODULE_3__.AxesHelper(5);
        this.scene.add(axesHelper);
        const ambientLight = new three__WEBPACK_IMPORTED_MODULE_3__.AmbientLight(0xffffff);
        ambientLight.intensity = 2;
        this.scene.add(ambientLight);
    }
    initSkeleton() {
        // tslint:disable-next-line: forin
        if (this.coordinateService.connections == null || this.coordinateService.lastPose == null) {
            return;
        }
        for (const point in this.coordinateService.lastPose) {
            const geometry = new three__WEBPACK_IMPORTED_MODULE_3__.SphereBufferGeometry(0.1, 32, 32);
            const material = new three__WEBPACK_IMPORTED_MODULE_3__.MeshBasicMaterial({ color: 0xafaab9 });
            const dot = new three__WEBPACK_IMPORTED_MODULE_3__.Mesh(geometry, material);
            dot.name = point.toString();
            this.dots.push({ index: point, dot });
            this.scene.add(dot);
        }
        // tslint:disable-next-line: forin
        for (const start in this.coordinateService.connections) {
            // tslint:disable-next-line: forin
            for (const end of this.coordinateService.connections[start]) {
                const material = new three__WEBPACK_IMPORTED_MODULE_3__.LineBasicMaterial({
                    color: 0x3f51b5,
                    linewidth: 4.0
                });
                const geometry = new three__WEBPACK_IMPORTED_MODULE_3__.BufferGeometry();
                const line = new three__WEBPACK_IMPORTED_MODULE_3__.Line(geometry, material);
                this.lines.push(line);
                this.dotsMapping.push([start, end]);
                this.scene.add(line);
            }
        }
        this.isInitialized = true;
    }
    colorLines() {
        let material = new three__WEBPACK_IMPORTED_MODULE_3__.LineBasicMaterial({ linewidth: 4.0 });
        switch (true) {
            case (this.progression < 20):
                material.color = new three__WEBPACK_IMPORTED_MODULE_3__.Color(this.color[0]);
                break;
            case (this.progression < 40):
                material.color = new three__WEBPACK_IMPORTED_MODULE_3__.Color(this.color[1]);
                break;
            case (this.progression < 60):
                material.color = new three__WEBPACK_IMPORTED_MODULE_3__.Color(this.color[2]);
                break;
            case (this.progression < 80):
                material.color = new three__WEBPACK_IMPORTED_MODULE_3__.Color(this.color[3]);
                break;
            default:
                material.color = new three__WEBPACK_IMPORTED_MODULE_3__.Color(this.color[4]);
                break;
        }
        this.lines.forEach(line => {
            line.material = material;
        });
    }
    renderSkeleton(scalar, offsetX, offsetY, offsetZ) {
        let pose = this.recordingPose;
        for (const dot of this.dots) {
            dot.dot.position.setX(pose[dot.index].x * scalar + offsetX);
            dot.dot.position.setY(pose[dot.index].y * scalar + offsetY);
            dot.dot.position.setZ(pose[dot.index].z * scalar + offsetZ);
        }
        // tslint:disable-next-line: forin
        for (const index in this.lines) {
            const mapping = this.dotsMapping[index];
            const line = this.lines[index];
            const x0 = pose[mapping[0]].x * scalar + offsetX;
            const x1 = pose[mapping[1]].x * scalar + offsetX;
            const y0 = pose[mapping[0]].y * scalar + offsetY;
            const y1 = pose[mapping[1]].y * scalar + offsetY;
            const z0 = pose[mapping[0]].z * scalar + offsetZ;
            const z1 = pose[mapping[1]].z * scalar + offsetZ;
            const start = new three__WEBPACK_IMPORTED_MODULE_3__.Vector3(x0, y0, z0);
            const end = new three__WEBPACK_IMPORTED_MODULE_3__.Vector3(x1, y1, z1);
            line.geometry.setFromPoints([start, end]);
            line.geometry.computeBoundingBox();
            line.geometry.computeBoundingSphere();
        }
    }
    selectJoint() {
        this.raycaster.setFromCamera(this.mouse, this.camera);
        const mouseIntersection = this.raycaster.intersectObjects(this.scene.children);
        let go = '';
        for (const intersected of mouseIntersection) {
            if (intersected.object instanceof three__WEBPACK_IMPORTED_MODULE_3__.Mesh) {
                go = intersected.object.name;
            }
        }
        if (go.length >= 1) {
            this.selectedJoint = go;
        }
        else {
            this.selectedJoint = '';
        }
    }
}
RendererComponent.ɵfac = function RendererComponent_Factory(t) { return new (t || RendererComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵdirectiveInject"](_services_data_service__WEBPACK_IMPORTED_MODULE_1__.DataService), _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵdirectiveInject"](_services_coordinate_service__WEBPACK_IMPORTED_MODULE_2__.CoordinateService), _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵdirectiveInject"](_angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_5__.MatSnackBar)); };
RendererComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵdefineComponent"]({ type: RendererComponent, selectors: [["app-renderer"]], viewQuery: function RendererComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵviewQuery"](_c0, 5);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵloadQuery"]()) && (ctx.editorContainer = _t.first);
    } }, hostBindings: function RendererComponent_HostBindings(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵlistener"]("resize", function RendererComponent_resize_HostBindingHandler($event) { return ctx.onWindowResize($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵresolveWindow"])("mousemove", function RendererComponent_mousemove_HostBindingHandler($event) { return ctx.onDocumentMouseMove($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵresolveDocument"])("touchend", function RendererComponent_touchend_HostBindingHandler($event) { return ctx.onTouchEnd($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵresolveDocument"]);
    } }, inputs: { currentExercise: "currentExercise" }, decls: 6, vars: 3, consts: [["autoplay", "", "loop", "", "oncanplay", "this.play()", "onloadedmetadata", "this.muted = true", 1, "exercise-video", 3, "currentTime"], ["src", "../../../assets/1.mp4", "type", "video/mp4"], [1, "reference-frame"], ["id", "editorContainer", 1, "div-editor", 2, "background", "transparent"], ["editorContainer", ""]], template: function RendererComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelementStart"](0, "video", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelement"](1, "source", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelementStart"](2, "div", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵtext"](3);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelement"](4, "div", 3, 4);
    } if (rf & 2) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵproperty"]("currentTime", ctx.frame / 1000);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵadvance"](3);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵtextInterpolate2"]("", ctx.frame, ", ", ctx.progression, "");
    } }, styles: [".div-editor[_ngcontent-%COMP%] {\n  position: absolute;\n  display: flex;\n  top: 0;\n  right: 0;\n}\n\n[_nghost-%COMP%]    >  >  > .mat-figure[_ngcontent-%COMP%] {\n  align-items: start !important;\n}\n\n.exercise-video[_ngcontent-%COMP%] {\n  position: fixed;\n  padding: 0;\n  margin: 0;\n  top: 0;\n  left: 0;\n  width: 100%;\n  height: 100%;\n}\n\n.mat-icon[_ngcontent-%COMP%] {\n  vertical-align: middle;\n}\n\n.speed-span[_ngcontent-%COMP%] {\n  margin-left: 0px;\n}\n\n.mat-card[_ngcontent-%COMP%] {\n  width: 100%;\n}\n\n.reference-frame[_ngcontent-%COMP%] {\n  margin: 0;\n  top: 50%;\n  left: 50%;\n  position: absolute;\n  transform: translate(-50%, -50%);\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInJlbmRlcmVyLmNvbXBvbmVudC5zY3NzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBO0VBQ0Usa0JBQUE7RUFDQSxhQUFBO0VBQ0EsTUFBQTtFQUNBLFFBQUE7QUFDRjs7QUFFQTtFQUNFLDZCQUFBO0FBQ0Y7O0FBRUE7RUFDRSxlQUFBO0VBQ0EsVUFBQTtFQUNBLFNBQUE7RUFDQSxNQUFBO0VBQ0EsT0FBQTtFQUNBLFdBQUE7RUFDQSxZQUFBO0FBQ0Y7O0FBRUE7RUFDRSxzQkFBQTtBQUNGOztBQUVBO0VBQ0UsZ0JBQUE7QUFDRjs7QUFFQTtFQUNFLFdBQUE7QUFDRjs7QUFFQTtFQUNFLFNBQUE7RUFDQSxRQUFBO0VBQ0EsU0FBQTtFQUNBLGtCQUFBO0VBRUEsZ0NBQUE7QUFDRiIsImZpbGUiOiJyZW5kZXJlci5jb21wb25lbnQuc2NzcyIsInNvdXJjZXNDb250ZW50IjpbIi5kaXYtZWRpdG9yIHtcbiAgcG9zaXRpb246IGFic29sdXRlO1xuICBkaXNwbGF5OiBmbGV4O1xuICB0b3A6IDA7XG4gIHJpZ2h0OiAwO1xufVxuXG46aG9zdCA+Pj4gLm1hdC1maWd1cmV7XG4gIGFsaWduLWl0ZW1zOiBzdGFydCAhaW1wb3J0YW50O1xufVxuXG4uZXhlcmNpc2UtdmlkZW8ge1xuICBwb3NpdGlvbjogZml4ZWQ7XG4gIHBhZGRpbmc6IDA7XG4gIG1hcmdpbjogMDtcbiAgdG9wOiAwO1xuICBsZWZ0OiAwO1xuICB3aWR0aDogMTAwJTtcbiAgaGVpZ2h0OiAxMDAlO1xufVxuXG4ubWF0LWljb24ge1xuICB2ZXJ0aWNhbC1hbGlnbjogbWlkZGxlO1xufVxuXG4uc3BlZWQtc3BhbiB7XG4gIG1hcmdpbi1sZWZ0OiAwcHg7XG59XG5cbi5tYXQtY2FyZCB7XG4gIHdpZHRoOiAxMDAlO1xufVxuXG4ucmVmZXJlbmNlLWZyYW1lIHtcbiAgbWFyZ2luOiAwO1xuICB0b3A6IDUwJTtcbiAgbGVmdDogNTAlO1xuICBwb3NpdGlvbjogYWJzb2x1dGU7XG4gIC1tcy10cmFuc2Zvcm06IHRyYW5zbGF0ZSgtNTAlLCAtNTAlKTtcbiAgdHJhbnNmb3JtOiB0cmFuc2xhdGUoLTUwJSwgLTUwJSk7XG59XG5cbiJdfQ== */"] });


/***/ }),

/***/ 3710:
/*!*************************************************!*\
  !*** ./src/app/step/select/select.component.ts ***!
  \*************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "SelectComponent": () => (/* binding */ SelectComponent)
/* harmony export */ });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ 2316);

class SelectComponent {
    constructor() { }
    ngOnInit() {
    }
}
SelectComponent.ɵfac = function SelectComponent_Factory(t) { return new (t || SelectComponent)(); };
SelectComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineComponent"]({ type: SelectComponent, selectors: [["app-select"]], decls: 2, vars: 0, template: function SelectComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "p");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](1, "select works!");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    } }, styles: [".header[_ngcontent-%COMP%] {\n  margin: 0;\n  top: 50%;\n  left: 50%;\n  position: absolute;\n  transform: translate(-50%, -50%);\n}\n\n.container[_ngcontent-%COMP%] {\n  position: fixed;\n  padding: 0;\n  margin: 0;\n  top: 0;\n  left: 0;\n  width: 100%;\n  height: 100%;\n  background: lightgray;\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInNlbGVjdC5jb21wb25lbnQuc2NzcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtFQUNFLFNBQUE7RUFDQSxRQUFBO0VBQ0EsU0FBQTtFQUNBLGtCQUFBO0VBRUEsZ0NBQUE7QUFDRjs7QUFFQTtFQUNFLGVBQUE7RUFDQSxVQUFBO0VBQ0EsU0FBQTtFQUNBLE1BQUE7RUFDQSxPQUFBO0VBQ0EsV0FBQTtFQUNBLFlBQUE7RUFDQSxxQkFBQTtBQUNGIiwiZmlsZSI6InNlbGVjdC5jb21wb25lbnQuc2NzcyIsInNvdXJjZXNDb250ZW50IjpbIi5oZWFkZXIge1xuICBtYXJnaW46IDA7XG4gIHRvcDogNTAlO1xuICBsZWZ0OiA1MCU7XG4gIHBvc2l0aW9uOiBhYnNvbHV0ZTtcbiAgLW1zLXRyYW5zZm9ybTogdHJhbnNsYXRlKC01MCUsIC01MCUpO1xuICB0cmFuc2Zvcm06IHRyYW5zbGF0ZSgtNTAlLCAtNTAlKTtcbn1cblxuLmNvbnRhaW5lciB7XG4gIHBvc2l0aW9uOiBmaXhlZDtcbiAgcGFkZGluZzogMDtcbiAgbWFyZ2luOiAwO1xuICB0b3A6IDA7XG4gIGxlZnQ6IDA7XG4gIHdpZHRoOiAxMDAlO1xuICBoZWlnaHQ6IDEwMCU7XG4gIGJhY2tncm91bmQ6cmdiKDIxMSwyMTEsMjExKVxufVxuIl19 */"] });


/***/ }),

/***/ 6502:
/*!****************************************!*\
  !*** ./src/app/step/step.component.ts ***!
  \****************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "StepComponent": () => (/* binding */ StepComponent)
/* harmony export */ });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var howler__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! howler */ 7170);
/* harmony import */ var howler__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(howler__WEBPACK_IMPORTED_MODULE_0__);
/* harmony import */ var _services_speech_service__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../services/speech.service */ 1359);
/* harmony import */ var _angular_common__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! @angular/common */ 4364);
/* harmony import */ var _intro_intro_component__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./intro/intro.component */ 2338);
/* harmony import */ var _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./renderer/renderer.component */ 2979);







const _c0 = ["tabs"];
function StepComponent_div_0_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelementStart"](0, "div");
    _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelement"](1, "app-intro");
    _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelementEnd"]();
} }
function StepComponent_app_renderer_1_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelement"](0, "app-renderer");
} }
class StepComponent {
    constructor(speechService) {
        this.speechService = speechService;
        this.welcome = "";
        this.begin = "";
        this.exercise = "";
        this.result = "";
        this.zoomSound = new howler__WEBPACK_IMPORTED_MODULE_0__.Howl({
            src: ['../../assets/zoom.mp3']
        });
        this.AudioContext = window.AudioContext;
        this.active = "exercise";
    }
    ngOnInit() {
        //this.speechService.start();
    }
    ngAfterViewInit() {
        this.speechService.result.subscribe(val => {
            this.result = val;
        });
        this.speechService.welcome.subscribe(val => {
            this.active = 'intro';
            //this.playZoomSound();
        });
        this.speechService.exercise.subscribe(() => {
            this.active = 'exercise';
        });
        this.speechService.begin.subscribe(val => {
            this.begin = val;
        });
    }
    playZoomSound() {
        this.zoomSound.play();
    }
}
StepComponent.ɵfac = function StepComponent_Factory(t) { return new (t || StepComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵdirectiveInject"](_services_speech_service__WEBPACK_IMPORTED_MODULE_1__.SpeechService)); };
StepComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵdefineComponent"]({ type: StepComponent, selectors: [["app-step"]], viewQuery: function StepComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵviewQuery"](_c0, 5, _angular_core__WEBPACK_IMPORTED_MODULE_4__.ElementRef);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵloadQuery"]()) && (ctx.tab = _t.first);
    } }, decls: 2, vars: 2, consts: [[4, "ngIf"]], template: function StepComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵtemplate"](0, StepComponent_div_0_Template, 2, 0, "div", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵtemplate"](1, StepComponent_app_renderer_1_Template, 1, 0, "app-renderer", 0);
    } if (rf & 2) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵproperty"]("ngIf", ctx.active === "intro");
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵproperty"]("ngIf", ctx.active === "exercise");
    } }, directives: [_angular_common__WEBPACK_IMPORTED_MODULE_5__.NgIf, _intro_intro_component__WEBPACK_IMPORTED_MODULE_2__.IntroComponent, _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_3__.RendererComponent], styles: ["\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IiIsImZpbGUiOiJzdGVwLmNvbXBvbmVudC5zY3NzIn0= */"] });


/***/ }),

/***/ 2340:
/*!*****************************************!*\
  !*** ./src/environments/environment.ts ***!
  \*****************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "environment": () => (/* binding */ environment)
/* harmony export */ });
// This file can be replaced during build by using the `fileReplacements` array.
// `ng build` replaces `environment.ts` with `environment.prod.ts`.
// The list of file replacements can be found in `angular.json`.
const environment = {
    production: false
};
/*
 * For easier debugging in development mode, you can import the following file
 * to ignore zone related error stack frames such as `zone.run`, `zoneDelegate.invokeTask`.
 *
 * This import should be commented out in production mode because it will have a negative impact
 * on performance if an error is thrown.
 */
// import 'zone.js/plugins/zone-error';  // Included with Angular CLI.


/***/ }),

/***/ 4431:
/*!*********************!*\
  !*** ./src/main.ts ***!
  \*********************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony import */ var _angular_platform_browser__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/platform-browser */ 1570);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _app_app_module__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./app/app.module */ 6747);
/* harmony import */ var _environments_environment__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./environments/environment */ 2340);




if (_environments_environment__WEBPACK_IMPORTED_MODULE_1__.environment.production) {
    (0,_angular_core__WEBPACK_IMPORTED_MODULE_2__.enableProdMode)();
}
_angular_platform_browser__WEBPACK_IMPORTED_MODULE_3__.platformBrowser().bootstrapModule(_app_app_module__WEBPACK_IMPORTED_MODULE_0__.AppModule)
    .catch(err => console.error(err));


/***/ })

},
/******/ __webpack_require__ => { // webpackRuntimeModules
/******/ var __webpack_exec__ = (moduleId) => (__webpack_require__(__webpack_require__.s = moduleId))
/******/ __webpack_require__.O(0, ["vendor"], () => (__webpack_exec__(4431)));
/******/ var __webpack_exports__ = __webpack_require__.O();
/******/ }
]);
//# sourceMappingURL=main.js.map