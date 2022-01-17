(self["webpackChunkshowroom"] = self["webpackChunkshowroom"] || []).push([["main"],{

/***/ 98255:
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
webpackEmptyAsyncContext.id = 98255;
module.exports = webpackEmptyAsyncContext;

/***/ }),

/***/ 90158:
/*!***************************************!*\
  !*** ./src/app/app-routing.module.ts ***!
  \***************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "AppRoutingModule": () => (/* binding */ AppRoutingModule)
/* harmony export */ });
/* harmony import */ var _angular_router__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! @angular/router */ 71258);
/* harmony import */ var _step_intro_intro_component__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./step/intro/intro.component */ 11533);
/* harmony import */ var _step_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./step/renderer/renderer.component */ 92979);
/* harmony import */ var _step_select_select_component__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./step/select/select.component */ 53710);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/core */ 2316);






const routes = [
    { path: 'renderer', component: _step_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_1__.RendererComponent },
    { path: 'select', component: _step_select_select_component__WEBPACK_IMPORTED_MODULE_2__.SelectComponent },
    { path: 'intro', component: _step_intro_intro_component__WEBPACK_IMPORTED_MODULE_0__.IntroComponent }
];
class AppRoutingModule {
}
AppRoutingModule.ɵfac = function AppRoutingModule_Factory(t) { return new (t || AppRoutingModule)(); };
AppRoutingModule.ɵmod = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵdefineNgModule"]({ type: AppRoutingModule });
AppRoutingModule.ɵinj = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵdefineInjector"]({ imports: [[_angular_router__WEBPACK_IMPORTED_MODULE_4__.RouterModule.forRoot(routes)], _angular_router__WEBPACK_IMPORTED_MODULE_4__.RouterModule] });
(function () { (typeof ngJitMode === "undefined" || ngJitMode) && _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵsetNgModuleScope"](AppRoutingModule, { imports: [_angular_router__WEBPACK_IMPORTED_MODULE_4__.RouterModule], exports: [_angular_router__WEBPACK_IMPORTED_MODULE_4__.RouterModule] }); })();


/***/ }),

/***/ 55041:
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
/* harmony import */ var _step_step_component__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./step/step.component */ 66502);


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

/***/ 36747:
/*!*******************************!*\
  !*** ./src/app/app.module.ts ***!
  \*******************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "AppModule": () => (/* binding */ AppModule)
/* harmony export */ });
/* harmony import */ var _angular_platform_browser__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! @angular/platform-browser */ 71570);
/* harmony import */ var _app_routing_module__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./app-routing.module */ 90158);
/* harmony import */ var _app_component__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./app.component */ 55041);
/* harmony import */ var _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(/*! @angular/platform-browser/animations */ 20718);
/* harmony import */ var _step_step_component__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./step/step.component */ 66502);
/* harmony import */ var _step_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./step/renderer/renderer.component */ 92979);
/* harmony import */ var _step_intro_intro_component__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./step/intro/intro.component */ 11533);
/* harmony import */ var _angular_common_http__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(/*! @angular/common/http */ 53882);
/* harmony import */ var _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(/*! @angular/material/grid-list */ 85937);
/* harmony import */ var _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(/*! @angular/material/snack-bar */ 68456);
/* harmony import */ var _angular_material_tabs__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(/*! @angular/material/tabs */ 9348);
/* harmony import */ var _angular_material_core__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(/*! @angular/material/core */ 32220);
/* harmony import */ var _angular_material_card__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(/*! @angular/material/card */ 42118);
/* harmony import */ var _step_select_select_component__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ./step/select/select.component */ 53710);
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_15__ = __webpack_require__(/*! @angular/material/icon */ 52529);
/* harmony import */ var _angular_material_button__WEBPACK_IMPORTED_MODULE_16__ = __webpack_require__(/*! @angular/material/button */ 70781);
/* harmony import */ var ngx_echarts__WEBPACK_IMPORTED_MODULE_17__ = __webpack_require__(/*! ngx-echarts */ 45877);
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
            _angular_material_card__WEBPACK_IMPORTED_MODULE_14__.MatCardModule,
            _angular_material_icon__WEBPACK_IMPORTED_MODULE_15__.MatIconModule,
            _angular_material_button__WEBPACK_IMPORTED_MODULE_16__.MatButtonModule,
            ngx_echarts__WEBPACK_IMPORTED_MODULE_17__.NgxEchartsModule.forRoot({
                /**
                 * This will import all modules from echarts.
                 * If you only need custom modules,
                 * please refer to [Custom Build] section.
                 */
                echarts: () => __webpack_require__.e(/*! import() */ "node_modules_echarts_index_js").then(__webpack_require__.bind(__webpack_require__, /*! echarts */ 1480)), // or import('./path-to-my-custom-echarts')
            })
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
        _angular_material_card__WEBPACK_IMPORTED_MODULE_14__.MatCardModule,
        _angular_material_icon__WEBPACK_IMPORTED_MODULE_15__.MatIconModule,
        _angular_material_button__WEBPACK_IMPORTED_MODULE_16__.MatButtonModule, ngx_echarts__WEBPACK_IMPORTED_MODULE_17__.NgxEchartsModule] }); })();


/***/ }),

/***/ 52586:
/*!************************************************!*\
  !*** ./src/app/services/coordinate.service.ts ***!
  \************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "CoordinateService": () => (/* binding */ CoordinateService)
/* harmony export */ });
/* harmony import */ var rxjs__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! rxjs */ 76491);
/* harmony import */ var rxjs__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! rxjs */ 13413);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _data_service__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./data.service */ 52468);



class CoordinateService {
    constructor(dataService) {
        this.dataService = dataService;
        this.matrabs_used = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23];
        this.matrabs_labels = ['M_Hip', 'L_Hip', 'R_Hip', 'L_Back', 'L_Knee', 'R_Knee', 'M_Back', 'L_Ankle', 'R_Ankle', 'U_Back', 'L_Toes', 'R_Toes', 'Neck', 'L_Collarbone', 'R_Collarbone', 'Head', 'L_Shoulder', 'R_Shoulder', 'L_Elbow', 'R_Elbow', 'L_Wrist', 'R_Wrist', 'L_Fingers', 'R_Fingers'];
        this.matrabs = [[1, 4], [1, 0], [2, 5], [2, 0], [3, 6], [3, 0], [4, 7], [5, 8], [6, 9], [7, 10], [8, 11], [9, 12], [12, 13], [12, 14], [12, 15], [13, 16], [14, 17], [16, 18], [17, 19], [18, 20], [19, 21], [20, 22], [21, 23]];
        this.connections = {};
        this.socket = dataService.socket;
        this.socket.subscribe(message => {
            if (message.usage == null && message.topic == null) {
                this.lastPose = message;
                this.update.next(this.lastPose);
            }
            else if (message.usage && message.usage === "reference_progress") {
                this.progression.next(message.data.data);
            }
            else if (message.usage && message.usage === "reference_frame") {
                this.frame.next(message.data);
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

/***/ 52468:
/*!******************************************!*\
  !*** ./src/app/services/data.service.ts ***!
  \******************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "DataService": () => (/* binding */ DataService)
/* harmony export */ });
/* harmony import */ var rxjs_operators__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! rxjs/operators */ 20088);
/* harmony import */ var rxjs_webSocket__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! rxjs/webSocket */ 27717);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _angular_common_http__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/common/http */ 53882);




class DataService {
    constructor(http) {
        this.http = http;
        this.socket = this.getWebsocket();
        this.exerciseSocket = (0,rxjs_webSocket__WEBPACK_IMPORTED_MODULE_0__.webSocket)('ws://localhost:3030');
        this.exerciseSocket.subscribe(val => console.log("WS", val));
    }
    getWebsocket() {
        return (0,rxjs_webSocket__WEBPACK_IMPORTED_MODULE_0__.webSocket)('ws://localhost:6161');
    }
    activateStationAndSetExercise(statiionID, exerciseName) {
        let msg = `{"id": "test","type": 0,"request": 1,"payload": {"station": ${statiionID},"exercise": -1}}`;
        let gejsoned = JSON.parse(msg);
        this.exerciseSocket.next(gejsoned);
        this.exerciseSocket.pipe((0,rxjs_operators__WEBPACK_IMPORTED_MODULE_1__.first)()).subscribe(val => {
            this.userID = val.id;
            let exercsieActivateMsg = `{"id": "${this.userID}","type": 0,"request": 3,"payload": {"station": ${statiionID},"exercise": ${exerciseName},"set_id": 1}}`;
            let gejsoned2 = JSON.parse(exercsieActivateMsg);
            this.exerciseSocket.next(gejsoned2);
        });
    }
    deactivateStationAndSetExercise(statiionID, exerciseName) {
        let exerciseDeactivateMsg = `{    
        "id": "${this.userID}",    
        "type": 0,    
        "request": 4,    
        "payload": {
         "station": ${statiionID}, 
         "exercise": ${exerciseName},
         "set_id": 1
        }
      }`;
        this.exerciseSocket.next(exerciseDeactivateMsg);
        let msg = `{
      "id": "${this.userID}",
      "type": 0,
      "request": 2,
      "payload": {}
    }`;
        this.exerciseSocket.next(msg);
    }
    disconnectWs() {
        // socket.complete();
    }
    sendWs(exName) {
        //TODO: Station?
        let obj = {
            isActive: true,
            exerciseName: exName,
            stationID: 1
        };
        this.socket.next(obj);
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
DataService.ɵfac = function DataService_Factory(t) { return new (t || DataService)(_angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵinject"](_angular_common_http__WEBPACK_IMPORTED_MODULE_3__.HttpClient)); };
DataService.ɵprov = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵdefineInjectable"]({ token: DataService, factory: DataService.ɵfac, providedIn: 'root' });


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
/* harmony import */ var rxjs__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! rxjs */ 13413);
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

/***/ 11533:
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

/***/ 92979:
/*!*****************************************************!*\
  !*** ./src/app/step/renderer/renderer.component.ts ***!
  \*****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "RendererComponent": () => (/* binding */ RendererComponent)
/* harmony export */ });
/* harmony import */ var three__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! three */ 64986);
/* harmony import */ var three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! three/examples/jsm/controls/OrbitControls.js */ 36668);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _services_data_service__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../../services/data.service */ 52468);
/* harmony import */ var _services_coordinate_service__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ../../services/coordinate.service */ 52586);
/* harmony import */ var _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! @angular/material/snack-bar */ 68456);






const _c0 = ["editorContainer"];
class RendererComponent {
    constructor(dataService, coordinateService, _snackbar) {
        this.dataService = dataService;
        this.coordinateService = coordinateService;
        this._snackbar = _snackbar;
        this.data = [];
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
        // initialize chart options:
        this.options = {
            title: {
                text: 'Ausführungsfortschritt'
            },
            tooltip: {
                trigger: 'axis',
                formatter: (params) => {
                    return "hi";
                },
                axisPointer: {
                    animation: false
                }
            },
            xAxis: {
                type: 'value',
                splitLine: {
                    show: false
                }
            },
            yAxis: {
                type: 'value',
                boundaryGap: [0, '100%'],
                splitLine: {
                    show: false
                }
            },
            series: [{
                    name: 'Mocking Data',
                    type: 'line',
                    showSymbol: false,
                    hoverAnimation: false,
                    data: this.data
                }]
        };
        this.coordinateService.progression.subscribe(val => {
            console.log(val);
            this.data.push(val);
            // update series data:
            this.updateOptions = {
                series: [{
                        data: this.data
                    }]
            };
        });
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
        this.renderer.setSize(600, 600);
        this.animate();
        this.coordinateService.update.subscribe(pose => {
            if (!this.isInitialized) {
                this.initSkeleton();
                return;
            }
            else if (pose) {
                this.recordingPose = pose;
                this.renderSkeleton(1, 3, -2, 3);
            }
        });
        this.coordinateService.frame.subscribe(frame => {
            console.log("Frame: ", frame);
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
    } }, inputs: { source: "source" }, decls: 5, vars: 2, consts: [["loop", "", "autoplay", "", "onloadedmetadata", "this.muted = true", 1, "exercise-video", 3, "currentTime"], ["video", ""], ["type", "video/mp4", 3, "src"], ["id", "editorContainer", 1, "div-editor", 2, "background", "transparent"], ["editorContainer", ""]], template: function RendererComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelementStart"](0, "video", 0, 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelement"](2, "source", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵelement"](3, "div", 3, 4);
    } if (rf & 2) {
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵproperty"]("currentTime", ctx.frame / 1000);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵadvance"](2);
        _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵproperty"]("src", ctx.source, _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵsanitizeUrl"]);
    } }, styles: [".div-editor[_ngcontent-%COMP%] {\n  position: absolute;\n  display: flex;\n  top: 0;\n  right: 0;\n}\n\n[_nghost-%COMP%]    >  >  > .mat-figure[_ngcontent-%COMP%] {\n  align-items: start !important;\n}\n\n.exercise-video[_ngcontent-%COMP%] {\n  position: fixed;\n  padding: 0;\n  margin: 0;\n  top: 0;\n  left: 0;\n  width: 100%;\n  height: 100%;\n}\n\n.mat-icon[_ngcontent-%COMP%] {\n  vertical-align: middle;\n}\n\n.speed-span[_ngcontent-%COMP%] {\n  margin-left: 0px;\n}\n\n.mat-card[_ngcontent-%COMP%] {\n  width: 100%;\n}\n\n.reference-frame[_ngcontent-%COMP%] {\n  margin: 0;\n  top: 50%;\n  left: 50%;\n  position: absolute;\n  transform: translate(-50%, -50%);\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInJlbmRlcmVyLmNvbXBvbmVudC5zY3NzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBO0VBQ0Usa0JBQUE7RUFDQSxhQUFBO0VBQ0EsTUFBQTtFQUNBLFFBQUE7QUFDRjs7QUFFQTtFQUNFLDZCQUFBO0FBQ0Y7O0FBRUE7RUFDRSxlQUFBO0VBQ0EsVUFBQTtFQUNBLFNBQUE7RUFDQSxNQUFBO0VBQ0EsT0FBQTtFQUNBLFdBQUE7RUFDQSxZQUFBO0FBQ0Y7O0FBRUE7RUFDRSxzQkFBQTtBQUNGOztBQUVBO0VBQ0UsZ0JBQUE7QUFDRjs7QUFFQTtFQUNFLFdBQUE7QUFDRjs7QUFFQTtFQUNFLFNBQUE7RUFDQSxRQUFBO0VBQ0EsU0FBQTtFQUNBLGtCQUFBO0VBRUEsZ0NBQUE7QUFDRjs7QUFFQTs7Ozs7O0dBQUEiLCJmaWxlIjoicmVuZGVyZXIuY29tcG9uZW50LnNjc3MiLCJzb3VyY2VzQ29udGVudCI6WyIuZGl2LWVkaXRvciB7XG4gIHBvc2l0aW9uOiBhYnNvbHV0ZTtcbiAgZGlzcGxheTogZmxleDtcbiAgdG9wOiAwO1xuICByaWdodDogMDtcbn1cblxuOmhvc3QgPj4+IC5tYXQtZmlndXJle1xuICBhbGlnbi1pdGVtczogc3RhcnQgIWltcG9ydGFudDtcbn1cblxuLmV4ZXJjaXNlLXZpZGVvIHtcbiAgcG9zaXRpb246IGZpeGVkO1xuICBwYWRkaW5nOiAwO1xuICBtYXJnaW46IDA7XG4gIHRvcDogMDtcbiAgbGVmdDogMDtcbiAgd2lkdGg6IDEwMCU7XG4gIGhlaWdodDogMTAwJTtcbn1cblxuLm1hdC1pY29uIHtcbiAgdmVydGljYWwtYWxpZ246IG1pZGRsZTtcbn1cblxuLnNwZWVkLXNwYW4ge1xuICBtYXJnaW4tbGVmdDogMHB4O1xufVxuXG4ubWF0LWNhcmQge1xuICB3aWR0aDogMTAwJTtcbn1cblxuLnJlZmVyZW5jZS1mcmFtZSB7XG4gIG1hcmdpbjogMDtcbiAgdG9wOiA1MCU7XG4gIGxlZnQ6IDUwJTtcbiAgcG9zaXRpb246IGFic29sdXRlO1xuICAtbXMtdHJhbnNmb3JtOiB0cmFuc2xhdGUoLTUwJSwgLTUwJSk7XG4gIHRyYW5zZm9ybTogdHJhbnNsYXRlKC01MCUsIC01MCUpO1xufVxuXG4vKiAuZGVtby1jaGFydCB7XG4gIHBvc2l0aW9uOiBhYnNvbHV0ZTtcbiAgZGlzcGxheTogZmxleDtcbiAgYm90dG9tOiAzMDA7XG4gIHJpZ2h0OiAwO1xuICB6LWluZGV4OiAxNTtcbn0gKi9cbiJdfQ== */"] });


/***/ }),

/***/ 53710:
/*!*************************************************!*\
  !*** ./src/app/step/select/select.component.ts ***!
  \*************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "SelectComponent": () => (/* binding */ SelectComponent)
/* harmony export */ });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var src_app_services_data_service__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! src/app/services/data.service */ 52468);
/* harmony import */ var _angular_common__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/common */ 54364);
/* harmony import */ var _angular_material_card__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/material/card */ 42118);
/* harmony import */ var _angular_material_button__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! @angular/material/button */ 70781);
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! @angular/material/icon */ 52529);







function SelectComponent_mat_card_1_mat_card_content_7_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](0, "mat-card-content");
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](1, "p");
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵtext"](2);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
} if (rf & 2) {
    const ex_r1 = _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵnextContext"]().$implicit;
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵadvance"](2);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵtextInterpolate1"](" ", ex_r1.descr, " ");
} }
function SelectComponent_mat_card_1_Template(rf, ctx) { if (rf & 1) {
    const _r5 = _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵgetCurrentView"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](0, "mat-card", 2);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](1, "mat-card-header");
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](2, "mat-card-title");
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵtext"](3);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelement"](4, "div", 3);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](5, "video", 4);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelement"](6, "source", 5);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵtemplate"](7, SelectComponent_mat_card_1_mat_card_content_7_Template, 3, 1, "mat-card-content", 6);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](8, "mat-card-actions", 7);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](9, "button", 8);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵlistener"]("click", function SelectComponent_mat_card_1_Template_button_click_9_listener() { const restoredCtx = _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵrestoreView"](_r5); const ex_r1 = restoredCtx.$implicit; const ctx_r4 = _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵnextContext"](); return ctx_r4.select(ex_r1); });
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵtext"](10, "Start ");
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](11, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵtext"](12, "play_arrow");
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
} if (rf & 2) {
    const ex_r1 = ctx.$implicit;
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵadvance"](3);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵtextInterpolate"](ex_r1.name);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵadvance"](3);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵproperty"]("src", ex_r1.src, _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵsanitizeUrl"]);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵproperty"]("ngIf", ex_r1.descr);
} }
class SelectComponent {
    constructor(dataService) {
        this.dataService = dataService;
        this.exercises = [
            { name: 'Military Press', descr: 'Set up by taking a barbell or other weight and putting it in a racking position. The weight is then pressed to overhead.', src: '../../../assets/military_press.mp4', mn: '229', stationID: 5 },
            { name: 'Deadlift', descr: 'The deadlift is a weight training exercise in which a loaded barbell or bar is lifted off the ground to the level of the hips, torso perpendicular to the floor, before being placed back on the ground.', src: '../../../assets/deadlift.mp4', mn: '105', stationID: 6 },
            { name: 'Squats', descr: 'A squat is a strength exercise in which the trainee lowers their hips from a standing position and then stands back up. ', src: '../../../assets/squats.mp4', mn: '111', stationID: 4 }
        ];
        this.exercise = new _angular_core__WEBPACK_IMPORTED_MODULE_1__.EventEmitter();
    }
    ngOnInit() {
    }
    select(ex) {
        this.exercise.next(ex);
    }
}
SelectComponent.ɵfac = function SelectComponent_Factory(t) { return new (t || SelectComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵdirectiveInject"](src_app_services_data_service__WEBPACK_IMPORTED_MODULE_0__.DataService)); };
SelectComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵdefineComponent"]({ type: SelectComponent, selectors: [["app-select"]], outputs: { exercise: "exercise" }, decls: 2, vars: 1, consts: [[1, "exercise-cards"], ["class", "exercise-card", 4, "ngFor", "ngForOf"], [1, "exercise-card"], ["mat-card-avatar", "", 1, "exercise-header-image"], ["autoplay", "", "loop", "", "oncanplay", "this.play()", "onloadedmetadata", "this.muted = true", "mat-card-image", ""], ["type", "video/mp4", 3, "src"], [4, "ngIf"], ["align", "end", 1, "bottom"], ["mat-button", "", 3, "click"]], template: function SelectComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementStart"](0, "div", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵtemplate"](1, SelectComponent_mat_card_1_Template, 13, 3, "mat-card", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵelementEnd"]();
    } if (rf & 2) {
        _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵproperty"]("ngForOf", ctx.exercises);
    } }, directives: [_angular_common__WEBPACK_IMPORTED_MODULE_2__.NgForOf, _angular_material_card__WEBPACK_IMPORTED_MODULE_3__.MatCard, _angular_material_card__WEBPACK_IMPORTED_MODULE_3__.MatCardHeader, _angular_material_card__WEBPACK_IMPORTED_MODULE_3__.MatCardTitle, _angular_material_card__WEBPACK_IMPORTED_MODULE_3__.MatCardAvatar, _angular_material_card__WEBPACK_IMPORTED_MODULE_3__.MatCardImage, _angular_common__WEBPACK_IMPORTED_MODULE_2__.NgIf, _angular_material_card__WEBPACK_IMPORTED_MODULE_3__.MatCardActions, _angular_material_button__WEBPACK_IMPORTED_MODULE_4__.MatButton, _angular_material_icon__WEBPACK_IMPORTED_MODULE_5__.MatIcon, _angular_material_card__WEBPACK_IMPORTED_MODULE_3__.MatCardContent], styles: [".header[_ngcontent-%COMP%] {\n  margin: 0;\n  top: 50%;\n  left: 50%;\n  position: absolute;\n  transform: translate(-50%, -50%);\n}\n\n.container[_ngcontent-%COMP%] {\n  position: fixed;\n  padding: 0;\n  margin: 0;\n  top: 0;\n  left: 0;\n  width: 100%;\n  height: 100%;\n  background: lightgray;\n}\n\n.exercise-cards[_ngcontent-%COMP%] {\n  margin: 0 auto;\n  text-align: center;\n  height: 100%;\n  width: 100%;\n  display: flex;\n  justify-content: center;\n  align-items: center;\n}\n\n.exercise-card[_ngcontent-%COMP%] {\n  text-align: left;\n  height: 400px;\n  max-width: 300px;\n  margin: 16px;\n  box-shadow: 0 5px 5px -3px rgba(0, 0, 0, 0.2), 0 8px 10px 1px rgba(0, 0, 0, 0.14), 0 3px 14px 2px rgba(0, 0, 0, 0.12);\n}\n\n.exercise-header-image[_ngcontent-%COMP%] {\n  background-image: url('gymy_logo.jpg');\n  background-size: cover;\n}\n\n.bottom[_ngcontent-%COMP%] {\n  position: absolute;\n  right: 25px;\n  bottom: 25px;\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInNlbGVjdC5jb21wb25lbnQuc2NzcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtFQUNFLFNBQUE7RUFDQSxRQUFBO0VBQ0EsU0FBQTtFQUNBLGtCQUFBO0VBRUEsZ0NBQUE7QUFDRjs7QUFFQTtFQUNFLGVBQUE7RUFDQSxVQUFBO0VBQ0EsU0FBQTtFQUNBLE1BQUE7RUFDQSxPQUFBO0VBQ0EsV0FBQTtFQUNBLFlBQUE7RUFDQSxxQkFBQTtBQUNGOztBQUVBO0VBQ0UsY0FBQTtFQUNBLGtCQUFBO0VBQ0EsWUFBQTtFQUNBLFdBQUE7RUFDQSxhQUFBO0VBQ0EsdUJBQUE7RUFDQSxtQkFBQTtBQUNGOztBQUVBO0VBQ0UsZ0JBQUE7RUFDQSxhQUFBO0VBQ0EsZ0JBQUE7RUFDQSxZQUFBO0VBQ0EscUhBQUE7QUFDRjs7QUFJQTtFQUNFLHNDQUFBO0VBQ0Esc0JBQUE7QUFERjs7QUFJQTtFQUNFLGtCQUFBO0VBQ0EsV0FBQTtFQUNBLFlBQUE7QUFERiIsImZpbGUiOiJzZWxlY3QuY29tcG9uZW50LnNjc3MiLCJzb3VyY2VzQ29udGVudCI6WyIuaGVhZGVyIHtcbiAgbWFyZ2luOiAwO1xuICB0b3A6IDUwJTtcbiAgbGVmdDogNTAlO1xuICBwb3NpdGlvbjogYWJzb2x1dGU7XG4gIC1tcy10cmFuc2Zvcm06IHRyYW5zbGF0ZSgtNTAlLCAtNTAlKTtcbiAgdHJhbnNmb3JtOiB0cmFuc2xhdGUoLTUwJSwgLTUwJSk7XG59XG5cbi5jb250YWluZXIge1xuICBwb3NpdGlvbjogZml4ZWQ7XG4gIHBhZGRpbmc6IDA7XG4gIG1hcmdpbjogMDtcbiAgdG9wOiAwO1xuICBsZWZ0OiAwO1xuICB3aWR0aDogMTAwJTtcbiAgaGVpZ2h0OiAxMDAlO1xuICBiYWNrZ3JvdW5kOnJnYigyMTEsMjExLDIxMSlcbn1cblxuLmV4ZXJjaXNlLWNhcmRzIHtcbiAgbWFyZ2luOiAwIGF1dG87XG4gIHRleHQtYWxpZ246IGNlbnRlcjtcbiAgaGVpZ2h0OiAxMDAlO1xuICB3aWR0aDogMTAwJTtcbiAgZGlzcGxheTogZmxleDtcbiAganVzdGlmeS1jb250ZW50OiBjZW50ZXI7XG4gIGFsaWduLWl0ZW1zOiBjZW50ZXI7XG59XG5cbi5leGVyY2lzZS1jYXJkIHtcbiAgdGV4dC1hbGlnbjogbGVmdDtcbiAgaGVpZ2h0OiA0MDBweDtcbiAgbWF4LXdpZHRoOiAzMDBweDtcbiAgbWFyZ2luOiAxNnB4O1xuICBib3gtc2hhZG93OiAwIDVweCA1cHggLTNweCByZ2JhKDAsIDAsIDAsIDAuMiksXG4gIDAgOHB4IDEwcHggMXB4IHJnYmEoMCwgMCwgMCwgMC4xNCksXG4gIDAgM3B4IDE0cHggMnB4IHJnYmEoMCwgMCwgMCwgMC4xMik7XG59XG5cbi5leGVyY2lzZS1oZWFkZXItaW1hZ2Uge1xuICBiYWNrZ3JvdW5kLWltYWdlOiB1cmwoLi4vLi4vLi4vYXNzZXRzL2d5bXlfbG9nby5qcGcpO1xuICBiYWNrZ3JvdW5kLXNpemU6IGNvdmVyO1xufVxuXG4uYm90dG9tIHtcbiAgcG9zaXRpb246IGFic29sdXRlO1xuICByaWdodDogMjVweDtcbiAgYm90dG9tOiAyNXB4O1xufVxuXG4iXX0= */"] });


/***/ }),

/***/ 66502:
/*!****************************************!*\
  !*** ./src/app/step/step.component.ts ***!
  \****************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "StepComponent": () => (/* binding */ StepComponent)
/* harmony export */ });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var howler__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! howler */ 77170);
/* harmony import */ var howler__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(howler__WEBPACK_IMPORTED_MODULE_0__);
/* harmony import */ var _services_speech_service__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../services/speech.service */ 1359);
/* harmony import */ var _services_data_service__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ../services/data.service */ 52468);
/* harmony import */ var _angular_common__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! @angular/common */ 54364);
/* harmony import */ var _select_select_component__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./select/select.component */ 53710);
/* harmony import */ var _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./renderer/renderer.component */ 92979);








const _c0 = ["tabs"];
function StepComponent_app_select_0_Template(rf, ctx) { if (rf & 1) {
    const _r3 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵgetCurrentView"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "app-select", 2);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("exercise", function StepComponent_app_select_0_Template_app_select_exercise_0_listener($event) { _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵrestoreView"](_r3); const ctx_r2 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵnextContext"](); return ctx_r2.selectedExercise($event); });
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} }
function StepComponent_app_renderer_1_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelement"](0, "app-renderer", 3);
} if (rf & 2) {
    const ctx_r1 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵnextContext"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("source", ctx_r1.exercise.src);
} }
class StepComponent {
    constructor(speechService, dataService) {
        this.speechService = speechService;
        this.dataService = dataService;
        this.welcome = "";
        this.begin = "";
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
    selectedExercise(ex) {
        this.exercise = ex;
        this.dataService.activateStationAndSetExercise(ex.stationID, ex.mn);
    }
    deactivate() {
        var _a, _b;
        this.dataService.deactivateStationAndSetExercise((_a = this.exercise) === null || _a === void 0 ? void 0 : _a.stationID, (_b = this.exercise) === null || _b === void 0 ? void 0 : _b.mn);
    }
}
StepComponent.ɵfac = function StepComponent_Factory(t) { return new (t || StepComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵdirectiveInject"](_services_speech_service__WEBPACK_IMPORTED_MODULE_1__.SpeechService), _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵdirectiveInject"](_services_data_service__WEBPACK_IMPORTED_MODULE_2__.DataService)); };
StepComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵdefineComponent"]({ type: StepComponent, selectors: [["app-step"]], viewQuery: function StepComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵviewQuery"](_c0, 5, _angular_core__WEBPACK_IMPORTED_MODULE_5__.ElementRef);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵloadQuery"]()) && (ctx.tab = _t.first);
    } }, decls: 2, vars: 2, consts: [[3, "exercise", 4, "ngIf"], [3, "source", 4, "ngIf"], [3, "exercise"], [3, "source"]], template: function StepComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](0, StepComponent_app_select_0_Template, 1, 0, "app-select", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](1, StepComponent_app_renderer_1_Template, 1, 1, "app-renderer", 1);
    } if (rf & 2) {
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngIf", !ctx.exercise);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngIf", ctx.exercise);
    } }, directives: [_angular_common__WEBPACK_IMPORTED_MODULE_6__.NgIf, _select_select_component__WEBPACK_IMPORTED_MODULE_3__.SelectComponent, _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_4__.RendererComponent], styles: ["\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IiIsImZpbGUiOiJzdGVwLmNvbXBvbmVudC5zY3NzIn0= */"] });


/***/ }),

/***/ 92340:
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

/***/ 14431:
/*!*********************!*\
  !*** ./src/main.ts ***!
  \*********************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony import */ var _angular_platform_browser__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/platform-browser */ 71570);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/core */ 2316);
/* harmony import */ var _app_app_module__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./app/app.module */ 36747);
/* harmony import */ var _environments_environment__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./environments/environment */ 92340);




if (_environments_environment__WEBPACK_IMPORTED_MODULE_1__.environment.production) {
    (0,_angular_core__WEBPACK_IMPORTED_MODULE_2__.enableProdMode)();
}
_angular_platform_browser__WEBPACK_IMPORTED_MODULE_3__.platformBrowser().bootstrapModule(_app_app_module__WEBPACK_IMPORTED_MODULE_0__.AppModule)
    .catch(err => console.error(err));


/***/ })

},
/******/ __webpack_require__ => { // webpackRuntimeModules
/******/ var __webpack_exec__ = (moduleId) => (__webpack_require__(__webpack_require__.s = moduleId))
/******/ __webpack_require__.O(0, ["vendor"], () => (__webpack_exec__(14431)));
/******/ var __webpack_exports__ = __webpack_require__.O();
/******/ }
]);
//# sourceMappingURL=main.js.map