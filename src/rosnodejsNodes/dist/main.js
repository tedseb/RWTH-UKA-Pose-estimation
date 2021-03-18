(window["webpackJsonp"] = window["webpackJsonp"] || []).push([["main"],{

/***/ 0:
/*!***************************!*\
  !*** multi ./src/main.ts ***!
  \***************************/
/*! no static exports found */
/***/ (function(module, exports, __webpack_require__) {

module.exports = __webpack_require__(/*! /home/optimus/TrainerAI-System/guihmi/src/main.ts */"zUnb");


/***/ }),

/***/ "AytR":
/*!*****************************************!*\
  !*** ./src/environments/environment.ts ***!
  \*****************************************/
/*! exports provided: environment */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "environment", function() { return environment; });
// This file can be replaced during build by using the `fileReplacements` array.
// `ng build --prod` replaces `environment.ts` with `environment.prod.ts`.
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
// import 'zone.js/dist/zone-error';  // Included with Angular CLI.


/***/ }),

/***/ "EnSQ":
/*!******************************************!*\
  !*** ./src/app/services/data.service.ts ***!
  \******************************************/
/*! exports provided: DataService */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "DataService", function() { return DataService; });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ "fXoL");
/* harmony import */ var rxjs_webSocket__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! rxjs/webSocket */ "3uOa");
/* harmony import */ var _angular_common_http__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/common/http */ "tk/3");




class DataService {
    constructor(http) {
        this.http = http;
    }
    getWebsocket() {
        return Object(rxjs_webSocket__WEBPACK_IMPORTED_MODULE_1__["webSocket"])('ws://localhost:3000');
    }
    disconnectWs() {
        //socket.complete();
    }
    sendWs(msg) {
        //socket.next(msg);
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
        //return connections;
    }
    startRecording() {
        this.http.post('/api/exercise/recording/start', null).subscribe(val => {
        });
    }
    stopRecording() {
        return this.http.post('/api/exercise/recording/stop', null);
    }
    saveRecording(recording) {
        this.http.post('/api/expert/recording/save', recording).subscribe(val => { });
    }
}
DataService.ɵfac = function DataService_Factory(t) { return new (t || DataService)(_angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵinject"](_angular_common_http__WEBPACK_IMPORTED_MODULE_2__["HttpClient"])); };
DataService.ɵprov = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineInjectable"]({ token: DataService, factory: DataService.ɵfac, providedIn: 'root' });
/*@__PURE__*/ (function () { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵsetClassMetadata"](DataService, [{
        type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["Injectable"],
        args: [{
                providedIn: 'root'
            }]
    }], function () { return [{ type: _angular_common_http__WEBPACK_IMPORTED_MODULE_2__["HttpClient"] }]; }, null); })();


/***/ }),

/***/ "Hxsc":
/*!*************************************************!*\
  !*** ./src/app/services/coordinates.service.ts ***!
  \*************************************************/
/*! exports provided: CoordinatesService */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "CoordinatesService", function() { return CoordinatesService; });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ "fXoL");
/* harmony import */ var rxjs__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! rxjs */ "qCKp");
/* harmony import */ var _data_service__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./data.service */ "EnSQ");




class CoordinatesService {
    constructor(dataService) {
        this.dataService = dataService;
        //editor playback attributes
        this.recording = [];
        this.ri = 1;
        this.isPlaying = false;
        this.playbackSpeed = 1.0;
        this.timeouts = [];
        this.socket = dataService.getWebsocket();
        this.socket.subscribe(message => {
            this.lastPose = message;
            this.update.next(this.lastPose);
        });
        this.update = new rxjs__WEBPACK_IMPORTED_MODULE_1__["BehaviorSubject"](this.lastPose);
        this.play = new rxjs__WEBPACK_IMPORTED_MODULE_1__["BehaviorSubject"](this.lastPose);
        this.playPosition = new rxjs__WEBPACK_IMPORTED_MODULE_1__["BehaviorSubject"](this.ri);
        this.connections = {
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
        };
    }
    startRecording() {
        this.dataService.startRecording();
    }
    stopRecording() {
        this.dataService.stopRecording().subscribe(res => {
            this.recording = res;
            console.log(this.recording);
        });
    }
    saveRecording() {
        this.dataService.saveRecording(this.recording);
    }
    playingRecording() {
        if (this.ri === this.recording.length - 1) {
            this.ri = 1;
        }
        let time = this.recording[this.ri][0] - this.recording[this.ri - 1][0];
        if (this.recording) {
            let id = window.setTimeout(() => {
                if (this.isPlaying === true) {
                    this.playPosition.next(this.ri);
                    this.play.next(this.recording[this.ri][1]);
                    this.ri++;
                    this.playingRecording();
                }
            }, time * (1 / this.playbackSpeed));
            this.timeouts.push(id);
        }
    }
    playRecording() {
        this.isPlaying = true;
        this.playingRecording();
    }
    stopPlayingRecording() {
        this.isPlaying = false;
    }
    setPlaybackSpeed(playbackSpeed) {
        this.playbackSpeed = playbackSpeed;
    }
    clearTimeouts() {
        this.timeouts.forEach(id => {
            window.clearTimeout(id);
        });
    }
    setPlayPosition(val) {
        //this.clearTimeouts();
        this.ri = val;
        this.play.next(this.recording[this.ri][1]);
        // this.playRecording();
    }
}
CoordinatesService.ɵfac = function CoordinatesService_Factory(t) { return new (t || CoordinatesService)(_angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵinject"](_data_service__WEBPACK_IMPORTED_MODULE_2__["DataService"])); };
CoordinatesService.ɵprov = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineInjectable"]({ token: CoordinatesService, factory: CoordinatesService.ɵfac, providedIn: 'root' });
/*@__PURE__*/ (function () { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵsetClassMetadata"](CoordinatesService, [{
        type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["Injectable"],
        args: [{
                providedIn: 'root'
            }]
    }], function () { return [{ type: _data_service__WEBPACK_IMPORTED_MODULE_2__["DataService"] }]; }, null); })();


/***/ }),

/***/ "Sy1n":
/*!**********************************!*\
  !*** ./src/app/app.component.ts ***!
  \**********************************/
/*! exports provided: AppComponent */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "AppComponent", function() { return AppComponent; });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ "fXoL");
/* harmony import */ var _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/material/toolbar */ "/t3+");
/* harmony import */ var _angular_material_button__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/material/button */ "bTqV");
/* harmony import */ var _angular_router__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/router */ "tyNb");
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! @angular/material/icon */ "NFeN");






class AppComponent {
    constructor() {
        this.title = 'hmi';
    }
}
AppComponent.ɵfac = function AppComponent_Factory(t) { return new (t || AppComponent)(); };
AppComponent.ɵcmp = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineComponent"]({ type: AppComponent, selectors: [["app-root"]], decls: 16, vars: 0, consts: [["id", "toolbar-paragraph"], ["mat-button", "", "color", "primary", "routerLink", "/renderer"], ["mat-button", "", "color", "primary", "routerLink", "/editor"], [1, "toolbar-spacer"], ["mat-icon-button", "", "color", "primary", "aria-label", "Heart icon"], ["mat-icon-button", "", "color", "primary", "aria-label", "Share icon"]], template: function AppComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "p", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](1, "mat-toolbar");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](2, "span");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](3, "TrainerAI | EDK");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](4, "button", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](5, "Renderer");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](6, "button", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](7, "Editor");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelement"](8, "span", 3);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](9, "button", 4);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](10, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](11, "favorite");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](12, "button", 5);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](13, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](14, "share");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelement"](15, "router-outlet");
    } }, directives: [_angular_material_toolbar__WEBPACK_IMPORTED_MODULE_1__["MatToolbar"], _angular_material_button__WEBPACK_IMPORTED_MODULE_2__["MatButton"], _angular_router__WEBPACK_IMPORTED_MODULE_3__["RouterLink"], _angular_material_icon__WEBPACK_IMPORTED_MODULE_4__["MatIcon"], _angular_router__WEBPACK_IMPORTED_MODULE_3__["RouterOutlet"]], styles: ["\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IiIsImZpbGUiOiJhcHAuY29tcG9uZW50LnNjc3MifQ== */"] });
/*@__PURE__*/ (function () { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵsetClassMetadata"](AppComponent, [{
        type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["Component"],
        args: [{
                selector: 'app-root',
                templateUrl: './app.component.html',
                styleUrls: ['./app.component.scss']
            }]
    }], null, null); })();


/***/ }),

/***/ "T1/D":
/*!************************************************!*\
  !*** ./src/app/renderer/renderer.component.ts ***!
  \************************************************/
/*! exports provided: RendererComponent */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "RendererComponent", function() { return RendererComponent; });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ "fXoL");
/* harmony import */ var three__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! three */ "Womt");
/* harmony import */ var three_examples_jsm_loaders_GLTFLoader_js__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! three/examples/jsm/loaders/GLTFLoader.js */ "NK00");
/* harmony import */ var three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! three/examples/jsm/controls/OrbitControls.js */ "RyHr");
/* harmony import */ var three_examples_jsm_controls_TransformControls_js__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! three/examples/jsm/controls/TransformControls.js */ "sfEl");
/* harmony import */ var dat_gui__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! dat.gui */ "iZKT");
/* harmony import */ var _services_data_service__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! ../services/data.service */ "EnSQ");
/* harmony import */ var _services_coordinates_service__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! ../services/coordinates.service */ "Hxsc");









const _c0 = ["rendererContainer"];
class RendererComponent {
    constructor(dataService, coordinatesService) {
        this.dataService = dataService;
        this.coordinatesService = coordinatesService;
        // properties
        this.pixelRatio = window.devicePixelRatio;
        this.mouse = new three__WEBPACK_IMPORTED_MODULE_1__["Vector2"]();
        this.touch = new three__WEBPACK_IMPORTED_MODULE_1__["Vector2"]();
        // gltf loader
        this.loader = new three_examples_jsm_loaders_GLTFLoader_js__WEBPACK_IMPORTED_MODULE_2__["GLTFLoader"]();
        // configs
        this.skeletonChecked = false;
        this.orbit = true;
        this.animated = false;
        // skeleton
        this.dots = [];
        this.dotsMapping = [];
        this.lines = [];
        const width = window.innerWidth;
        const height = window.innerHeight - 64;
        this.renderer = new three__WEBPACK_IMPORTED_MODULE_1__["WebGLRenderer"]({ antialias: true });
        this.scene = new three__WEBPACK_IMPORTED_MODULE_1__["Scene"]();
        this.camera = new three__WEBPACK_IMPORTED_MODULE_1__["PerspectiveCamera"](25, width / height, 0.1, 1000);
        this.clock = new three__WEBPACK_IMPORTED_MODULE_1__["Clock"]();
        this.model = new three__WEBPACK_IMPORTED_MODULE_1__["Object3D"]();
        this.mixer = new three__WEBPACK_IMPORTED_MODULE_1__["AnimationMixer"](this.model);
        this.orbitControls = new three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_3__["OrbitControls"](this.camera, this.renderer.domElement);
        this.orbitControls.update();
        this.gui = new dat_gui__WEBPACK_IMPORTED_MODULE_5__["GUI"]({ autoPlace: false });
    }
    // event listeners
    onWindowResize(event) {
        const width = window.innerWidth;
        const height = window.innerHeight - 64;
        this.renderer.setSize(width, height);
        this.camera.aspect = width / height;
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
        var _a;
        (_a = document.getElementById('guiContainer')) === null || _a === void 0 ? void 0 : _a.appendChild(this.gui.domElement);
        this.initMenu();
        this.initScene();
    }
    ngAfterViewInit() {
        this.camera.lookAt(this.scene.position);
        this.rendererContainer.nativeElement.appendChild(this.renderer.domElement);
        this.renderer.setPixelRatio(this.pixelRatio);
        this.renderer.setSize(window.innerWidth, (window.innerHeight - 64));
        window.requestAnimationFrame(() => this.animate());
    }
    animate() {
        window.requestAnimationFrame(() => this.animate());
        const delta = this.clock.getDelta();
        if (this.mixer && this.animated) {
            this.mixer.update(delta);
        }
        this.renderer.render(this.scene, this.camera);
    }
    initScene() {
        this.renderer.setClearColor(0xffffff);
        this.camera.position.set(20, 20, 20);
        this.orbitControls.enablePan = true;
        this.orbitControls.enableZoom = true;
        this.orbitControls.enableKeys = true;
        const axesHelper = new three__WEBPACK_IMPORTED_MODULE_1__["AxesHelper"](5);
        this.scene.add(axesHelper);
        const ambientLight = new three__WEBPACK_IMPORTED_MODULE_1__["AmbientLight"](0xffffff);
        ambientLight.intensity = 2;
        this.scene.add(ambientLight);
    }
    initMenu() {
        const menu = {
            model: {},
            animate: false,
            skeleton: false,
            deleteBones: () => { this.deleteBones(); },
            edit: () => { this.addBoneTransformations(this.model); },
            reset: () => { this.reset(); },
            addMash: () => {
                const mesh = new three__WEBPACK_IMPORTED_MODULE_1__["Mesh"]();
            },
            renderSkeleton: () => { this.initSkeleton(); },
            startRecording: () => { this.coordinatesService.startRecording(); },
            stopRecording: () => { this.coordinatesService.stopRecording(); },
            saveRecording: () => { this.coordinatesService.saveRecording(); }
        };
        /*const modelswitch = this.gui.add(menu, 'model',
          {
            dancer: '../../assets/dancer/dancer.gltf',
            basic: '../../assets/basic/low_poly.glb',
            nathan: '../../assets/nathan/nathan.gltf',
            muscle: '../../assets/muschelmann/muschelmann.gltf'
          })
          .name('Model');
        modelswitch.onChange(val => this.loadModel(val));
        modelswitch.setValue('../../assets/dancer/dancer.gltf');*/
        /*const animateCheck = this.gui.add(menu, 'animate').name('Animate');
        animateCheck.onChange(val => {
          this.setAnimation(val);
        }); */
        //const skeletoncheck = this.gui.add(menu, 'skeleton', false).name('Show Skeleton');
        //skeletoncheck.onChange(val => { this.skeletonCheck(val); });
        const startRecording = this.gui.add(menu, 'startRecording').name('🎥');
        const stopRecording = this.gui.add(menu, 'stopRecording').name('🎬');
        const saveRecording = this.gui.add(menu, 'saveRecording').name('💾');
        //const deleteBones = this.gui.add(menu, 'deleteBones').name('Delete Not Used Bones');
        const resetButton = this.gui.add(menu, 'reset').name('Reset');
        //const editButton = this.gui.add(menu, 'edit').name('Edit');
        const renderSceleton = this.gui.add(menu, 'renderSkeleton').name('Render Skeleton');
    }
    deleteBones() {
        this.model.traverse(obj => {
            if (obj instanceof three__WEBPACK_IMPORTED_MODULE_1__["Bone"]) {
                if (obj.name.startsWith('f_') || obj.name.startsWith('thumb') || obj.name.startsWith('palm')) {
                    console.log("deleted");
                    this.scene.remove(obj);
                }
            }
        });
    }
    loadModel(path) {
        this.scene.remove(this.model);
        if (this.skeletonChecked) {
            this.skeletonCheck(true);
        }
        this.loader.load(path, gltf => {
            this.model = gltf.scene;
            this.mixer = new three__WEBPACK_IMPORTED_MODULE_1__["AnimationMixer"](gltf.scene);
            if (gltf.animations[0]) {
                const action = this.mixer.clipAction(gltf.animations[0]);
                action.play();
            }
            this.scene.add(gltf.scene);
        });
    }
    skeletonCheck(state) {
        if (state) {
            this.skeletonChecked = true;
            const skeletonHelper = new three__WEBPACK_IMPORTED_MODULE_1__["SkeletonHelper"](this.model);
            skeletonHelper.name = 'skeletonHelper';
            this.scene.add(skeletonHelper);
        }
        else {
            this.skeletonChecked = false;
            const skeletonHelper = this.scene.getObjectByName('skeletonHelper');
            if (skeletonHelper) {
                this.scene.remove(skeletonHelper);
            }
            this.animate();
        }
    }
    addBoneTransformations(model) {
        const modelBones = this.gui.addFolder('Model Bones');
        const dict = {};
        model.traverse(obj => {
            if (obj instanceof three__WEBPACK_IMPORTED_MODULE_1__["Bone"]) {
                console.log(obj.name);
                dict[obj.name] = () => { this.createTransformControl(this.model, obj.name); };
                modelBones.add(dict, obj.name);
            }
        });
    }
    createTransformControl(model, name) {
        const oldTransformControl = this.scene.getObjectByName('transformControl');
        if (oldTransformControl) {
            oldTransformControl.detach();
            this.scene.remove(oldTransformControl);
        }
        const transformControl = new three_examples_jsm_controls_TransformControls_js__WEBPACK_IMPORTED_MODULE_4__["TransformControls"](this.camera, this.renderer.domElement);
        // tslint:disable-next-line: no-non-null-assertion
        transformControl.attach(model.getObjectByName(name));
        transformControl.name = 'transformControl';
        transformControl.mode = 'rotate';
        this.switchControls('transform');
        this.scene.add(transformControl);
    }
    switchControls(name) {
        switch (name) {
            case 'orbit':
                this.reset();
                break;
            case 'transform':
                this.orbit = false;
                this.orbitControls.enabled = false;
                break;
        }
    }
    setAnimation(state) {
        this.animated = state;
        if (!state) {
            if (this.mixer) {
                this.mixer.setTime(0);
            }
        }
    }
    reset() {
        const oldTransformControl = this.scene.getObjectByName('transformControl');
        if (oldTransformControl) {
            oldTransformControl.detach();
            this.scene.remove(oldTransformControl);
        }
        this.orbit = true;
        this.orbitControls.enabled = true;
        for (let line of this.lines) {
            this.scene.remove(line);
        }
        for (const dot of this.dots) {
            this.scene.remove(dot.dot);
        }
        this.lines = [];
        this.dots = [];
        this.scene.remove(this.model);
    }
    initSkeleton() {
        this.reset();
        // tslint:disable-next-line: forin
        for (const point in this.coordinatesService.lastPose) {
            console.log(point);
            const geometry = new three__WEBPACK_IMPORTED_MODULE_1__["SphereBufferGeometry"](0.1, 32, 32);
            const material = new three__WEBPACK_IMPORTED_MODULE_1__["MeshBasicMaterial"]({ color: 0xafaab9 });
            const dot = new three__WEBPACK_IMPORTED_MODULE_1__["Mesh"](geometry, material);
            dot.name = point.toString();
            this.dots.push({ index: point, dot });
            this.scene.add(dot);
        }
        // tslint:disable-next-line: forin
        for (const start in this.coordinatesService.connections) {
            // tslint:disable-next-line: forin
            for (const end of this.coordinatesService.connections[start]) {
                const material = new three__WEBPACK_IMPORTED_MODULE_1__["LineBasicMaterial"]({
                    color: 0x0000ff,
                    linewidth: 4.0
                });
                const geometry = new three__WEBPACK_IMPORTED_MODULE_1__["BufferGeometry"]();
                const line = new three__WEBPACK_IMPORTED_MODULE_1__["Line"](geometry, material);
                this.lines.push(line);
                this.dotsMapping.push([start, end]);
                this.scene.add(line);
            }
        }
        this.renderSkeleton(5, 0, 0, 0);
        this.coordinatesService.update.subscribe(() => {
            //this.transferCoordinates();
            this.renderSkeleton(5, 0, 0, 0);
        });
    }
    transferCoordinates(boneName, coordinate) {
    }
    renderSkeleton(scalar, offsetX, offsetY, offsetZ) {
        const pose = this.coordinatesService.lastPose;
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
            const start = new three__WEBPACK_IMPORTED_MODULE_1__["Vector3"](x0, y0, z0);
            const end = new three__WEBPACK_IMPORTED_MODULE_1__["Vector3"](x1, y1, z1);
            line.geometry.setFromPoints([start, end]);
            line.geometry.computeBoundingBox();
            line.geometry.computeBoundingSphere();
        }
    }
}
RendererComponent.ɵfac = function RendererComponent_Factory(t) { return new (t || RendererComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdirectiveInject"](_services_data_service__WEBPACK_IMPORTED_MODULE_6__["DataService"]), _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdirectiveInject"](_services_coordinates_service__WEBPACK_IMPORTED_MODULE_7__["CoordinatesService"])); };
RendererComponent.ɵcmp = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineComponent"]({ type: RendererComponent, selectors: [["app-renderer"]], viewQuery: function RendererComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵviewQuery"](_c0, true);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵloadQuery"]()) && (ctx.rendererContainer = _t.first);
    } }, hostBindings: function RendererComponent_HostBindings(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("resize", function RendererComponent_resize_HostBindingHandler($event) { return ctx.onWindowResize($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵresolveWindow"])("mousemove", function RendererComponent_mousemove_HostBindingHandler($event) { return ctx.onDocumentMouseMove($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵresolveDocument"])("touchend", function RendererComponent_touchend_HostBindingHandler($event) { return ctx.onTouchEnd($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵresolveDocument"]);
    } }, decls: 3, vars: 0, consts: [["id", "rendererContainer", 1, "div-renderer"], ["rendererContainer", ""], ["id", "guiContainer"]], template: function RendererComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "div", 0, 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelement"](2, "div", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    } }, styles: [".div-renderer[_ngcontent-%COMP%] {\n  position: relative;\n  display: flex;\n  justify-content: center;\n}\n\n#guiContainer[_ngcontent-%COMP%] {\n  position: absolute;\n  top: 0em;\n  right: 0em;\n  z-index: 1;\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3JlbmRlcmVyLmNvbXBvbmVudC5zY3NzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBO0VBQ0Usa0JBQUE7RUFDQSxhQUFBO0VBQ0EsdUJBQUE7QUFDRjs7QUFFQTtFQUNFLGtCQUFBO0VBQ0EsUUFBQTtFQUNBLFVBQUE7RUFDQSxVQUFBO0FBQ0YiLCJmaWxlIjoicmVuZGVyZXIuY29tcG9uZW50LnNjc3MiLCJzb3VyY2VzQ29udGVudCI6WyIuZGl2LXJlbmRlcmVyIHtcbiAgcG9zaXRpb246IHJlbGF0aXZlO1xuICBkaXNwbGF5OiBmbGV4O1xuICBqdXN0aWZ5LWNvbnRlbnQ6IGNlbnRlcjtcbn1cblxuI2d1aUNvbnRhaW5lciB7XG4gIHBvc2l0aW9uOiBhYnNvbHV0ZTtcbiAgdG9wOiAwZW07XG4gIHJpZ2h0OiAwZW07XG4gIHotaW5kZXg6IDE7XG59XG4iXX0= */"] });
/*@__PURE__*/ (function () { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵsetClassMetadata"](RendererComponent, [{
        type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["Component"],
        args: [{
                selector: 'app-renderer',
                templateUrl: './renderer.component.html',
                styleUrls: ['./renderer.component.scss']
            }]
    }], function () { return [{ type: _services_data_service__WEBPACK_IMPORTED_MODULE_6__["DataService"] }, { type: _services_coordinates_service__WEBPACK_IMPORTED_MODULE_7__["CoordinatesService"] }]; }, { rendererContainer: [{
            type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["ViewChild"],
            args: ['rendererContainer']
        }], onWindowResize: [{
            type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["HostListener"],
            args: ['window:resize', ['$event']]
        }], onDocumentMouseMove: [{
            type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["HostListener"],
            args: ['document:mousemove', ['$event']]
        }], onTouchEnd: [{
            type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["HostListener"],
            args: ['document:touchend', ['$event']]
        }] }); })();


/***/ }),

/***/ "ZAI4":
/*!*******************************!*\
  !*** ./src/app/app.module.ts ***!
  \*******************************/
/*! exports provided: AppModule */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "AppModule", function() { return AppModule; });
/* harmony import */ var _angular_platform_browser__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/platform-browser */ "jhN1");
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/core */ "fXoL");
/* harmony import */ var _angular_common_http__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/common/http */ "tk/3");
/* harmony import */ var _app_routing_module__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./app-routing.module */ "vY5A");
/* harmony import */ var _app_component__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./app.component */ "Sy1n");
/* harmony import */ var _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ./renderer/renderer.component */ "T1/D");
/* harmony import */ var _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! @angular/platform-browser/animations */ "R1ws");
/* harmony import */ var _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! @angular/material/toolbar */ "/t3+");
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(/*! @angular/material/icon */ "NFeN");
/* harmony import */ var _angular_material_button__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(/*! @angular/material/button */ "bTqV");
/* harmony import */ var _angular_material_sidenav__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(/*! @angular/material/sidenav */ "XhcP");
/* harmony import */ var _angular_material_slide_toggle__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(/*! @angular/material/slide-toggle */ "1jcm");
/* harmony import */ var _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(/*! @angular/material/grid-list */ "zkoq");
/* harmony import */ var _angular_material_badge__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(/*! @angular/material/badge */ "TU8p");
/* harmony import */ var _angular_material_select__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(/*! @angular/material/select */ "d3UM");
/* harmony import */ var _angular_forms__WEBPACK_IMPORTED_MODULE_15__ = __webpack_require__(/*! @angular/forms */ "3Pt+");
/* harmony import */ var _angular_material_card__WEBPACK_IMPORTED_MODULE_16__ = __webpack_require__(/*! @angular/material/card */ "Wp6s");
/* harmony import */ var _editor_editor_component__WEBPACK_IMPORTED_MODULE_17__ = __webpack_require__(/*! ./editor/editor.component */ "xD4D");
/* harmony import */ var _angular_material_form_field__WEBPACK_IMPORTED_MODULE_18__ = __webpack_require__(/*! @angular/material/form-field */ "kmnG");
/* harmony import */ var _angular_material_input__WEBPACK_IMPORTED_MODULE_19__ = __webpack_require__(/*! @angular/material/input */ "qFsG");
/* harmony import */ var _angular_material_slider__WEBPACK_IMPORTED_MODULE_20__ = __webpack_require__(/*! @angular/material/slider */ "5RNC");
/* harmony import */ var _angular_material_menu__WEBPACK_IMPORTED_MODULE_21__ = __webpack_require__(/*! @angular/material/menu */ "STbY");
/* harmony import */ var _angular_material_chips__WEBPACK_IMPORTED_MODULE_22__ = __webpack_require__(/*! @angular/material/chips */ "A5z7");






// angular material module imports


















class AppModule {
}
AppModule.ɵmod = _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵdefineNgModule"]({ type: AppModule, bootstrap: [_app_component__WEBPACK_IMPORTED_MODULE_4__["AppComponent"]] });
AppModule.ɵinj = _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵdefineInjector"]({ factory: function AppModule_Factory(t) { return new (t || AppModule)(); }, providers: [], imports: [[
            _angular_forms__WEBPACK_IMPORTED_MODULE_15__["FormsModule"],
            _angular_platform_browser__WEBPACK_IMPORTED_MODULE_0__["BrowserModule"],
            _app_routing_module__WEBPACK_IMPORTED_MODULE_3__["AppRoutingModule"],
            _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_6__["BrowserAnimationsModule"],
            _angular_common_http__WEBPACK_IMPORTED_MODULE_2__["HttpClientModule"],
            _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_7__["MatToolbarModule"],
            _angular_material_icon__WEBPACK_IMPORTED_MODULE_8__["MatIconModule"],
            _angular_material_button__WEBPACK_IMPORTED_MODULE_9__["MatButtonModule"],
            _angular_material_sidenav__WEBPACK_IMPORTED_MODULE_10__["MatSidenavModule"],
            _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_12__["MatGridListModule"],
            _angular_material_card__WEBPACK_IMPORTED_MODULE_16__["MatCardModule"],
            _angular_material_slide_toggle__WEBPACK_IMPORTED_MODULE_11__["MatSlideToggleModule"],
            _angular_material_select__WEBPACK_IMPORTED_MODULE_14__["MatSelectModule"],
            _angular_material_badge__WEBPACK_IMPORTED_MODULE_13__["MatBadgeModule"],
            _angular_material_form_field__WEBPACK_IMPORTED_MODULE_18__["MatFormFieldModule"],
            _angular_material_input__WEBPACK_IMPORTED_MODULE_19__["MatInputModule"],
            _angular_material_slider__WEBPACK_IMPORTED_MODULE_20__["MatSliderModule"],
            _angular_material_chips__WEBPACK_IMPORTED_MODULE_22__["MatChipsModule"],
            _angular_material_menu__WEBPACK_IMPORTED_MODULE_21__["MatMenuModule"]
        ]] });
(function () { (typeof ngJitMode === "undefined" || ngJitMode) && _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵɵsetNgModuleScope"](AppModule, { declarations: [_app_component__WEBPACK_IMPORTED_MODULE_4__["AppComponent"],
        _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_5__["RendererComponent"],
        _editor_editor_component__WEBPACK_IMPORTED_MODULE_17__["EditorComponent"]], imports: [_angular_forms__WEBPACK_IMPORTED_MODULE_15__["FormsModule"],
        _angular_platform_browser__WEBPACK_IMPORTED_MODULE_0__["BrowserModule"],
        _app_routing_module__WEBPACK_IMPORTED_MODULE_3__["AppRoutingModule"],
        _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_6__["BrowserAnimationsModule"],
        _angular_common_http__WEBPACK_IMPORTED_MODULE_2__["HttpClientModule"],
        _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_7__["MatToolbarModule"],
        _angular_material_icon__WEBPACK_IMPORTED_MODULE_8__["MatIconModule"],
        _angular_material_button__WEBPACK_IMPORTED_MODULE_9__["MatButtonModule"],
        _angular_material_sidenav__WEBPACK_IMPORTED_MODULE_10__["MatSidenavModule"],
        _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_12__["MatGridListModule"],
        _angular_material_card__WEBPACK_IMPORTED_MODULE_16__["MatCardModule"],
        _angular_material_slide_toggle__WEBPACK_IMPORTED_MODULE_11__["MatSlideToggleModule"],
        _angular_material_select__WEBPACK_IMPORTED_MODULE_14__["MatSelectModule"],
        _angular_material_badge__WEBPACK_IMPORTED_MODULE_13__["MatBadgeModule"],
        _angular_material_form_field__WEBPACK_IMPORTED_MODULE_18__["MatFormFieldModule"],
        _angular_material_input__WEBPACK_IMPORTED_MODULE_19__["MatInputModule"],
        _angular_material_slider__WEBPACK_IMPORTED_MODULE_20__["MatSliderModule"],
        _angular_material_chips__WEBPACK_IMPORTED_MODULE_22__["MatChipsModule"],
        _angular_material_menu__WEBPACK_IMPORTED_MODULE_21__["MatMenuModule"]] }); })();
/*@__PURE__*/ (function () { _angular_core__WEBPACK_IMPORTED_MODULE_1__["ɵsetClassMetadata"](AppModule, [{
        type: _angular_core__WEBPACK_IMPORTED_MODULE_1__["NgModule"],
        args: [{
                declarations: [
                    _app_component__WEBPACK_IMPORTED_MODULE_4__["AppComponent"],
                    _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_5__["RendererComponent"],
                    _editor_editor_component__WEBPACK_IMPORTED_MODULE_17__["EditorComponent"]
                ],
                imports: [
                    _angular_forms__WEBPACK_IMPORTED_MODULE_15__["FormsModule"],
                    _angular_platform_browser__WEBPACK_IMPORTED_MODULE_0__["BrowserModule"],
                    _app_routing_module__WEBPACK_IMPORTED_MODULE_3__["AppRoutingModule"],
                    _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_6__["BrowserAnimationsModule"],
                    _angular_common_http__WEBPACK_IMPORTED_MODULE_2__["HttpClientModule"],
                    _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_7__["MatToolbarModule"],
                    _angular_material_icon__WEBPACK_IMPORTED_MODULE_8__["MatIconModule"],
                    _angular_material_button__WEBPACK_IMPORTED_MODULE_9__["MatButtonModule"],
                    _angular_material_sidenav__WEBPACK_IMPORTED_MODULE_10__["MatSidenavModule"],
                    _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_12__["MatGridListModule"],
                    _angular_material_card__WEBPACK_IMPORTED_MODULE_16__["MatCardModule"],
                    _angular_material_slide_toggle__WEBPACK_IMPORTED_MODULE_11__["MatSlideToggleModule"],
                    _angular_material_select__WEBPACK_IMPORTED_MODULE_14__["MatSelectModule"],
                    _angular_material_badge__WEBPACK_IMPORTED_MODULE_13__["MatBadgeModule"],
                    _angular_material_form_field__WEBPACK_IMPORTED_MODULE_18__["MatFormFieldModule"],
                    _angular_material_input__WEBPACK_IMPORTED_MODULE_19__["MatInputModule"],
                    _angular_material_slider__WEBPACK_IMPORTED_MODULE_20__["MatSliderModule"],
                    _angular_material_chips__WEBPACK_IMPORTED_MODULE_22__["MatChipsModule"],
                    _angular_material_menu__WEBPACK_IMPORTED_MODULE_21__["MatMenuModule"]
                ],
                providers: [],
                bootstrap: [_app_component__WEBPACK_IMPORTED_MODULE_4__["AppComponent"]]
            }]
    }], null, null); })();


/***/ }),

/***/ "vY5A":
/*!***************************************!*\
  !*** ./src/app/app-routing.module.ts ***!
  \***************************************/
/*! exports provided: AppRoutingModule */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "AppRoutingModule", function() { return AppRoutingModule; });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ "fXoL");
/* harmony import */ var _angular_router__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/router */ "tyNb");
/* harmony import */ var _editor_editor_component__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./editor/editor.component */ "xD4D");
/* harmony import */ var _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./renderer/renderer.component */ "T1/D");






const routes = [
    { path: 'editor', component: _editor_editor_component__WEBPACK_IMPORTED_MODULE_2__["EditorComponent"] },
    { path: 'renderer', component: _renderer_renderer_component__WEBPACK_IMPORTED_MODULE_3__["RendererComponent"] },
    { path: '', redirectTo: '/renderer', pathMatch: 'full' }
];
class AppRoutingModule {
}
AppRoutingModule.ɵmod = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineNgModule"]({ type: AppRoutingModule });
AppRoutingModule.ɵinj = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineInjector"]({ factory: function AppRoutingModule_Factory(t) { return new (t || AppRoutingModule)(); }, imports: [[_angular_router__WEBPACK_IMPORTED_MODULE_1__["RouterModule"].forRoot(routes, { useHash: true })], _angular_router__WEBPACK_IMPORTED_MODULE_1__["RouterModule"]] });
(function () { (typeof ngJitMode === "undefined" || ngJitMode) && _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵsetNgModuleScope"](AppRoutingModule, { imports: [_angular_router__WEBPACK_IMPORTED_MODULE_1__["RouterModule"]], exports: [_angular_router__WEBPACK_IMPORTED_MODULE_1__["RouterModule"]] }); })();
/*@__PURE__*/ (function () { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵsetClassMetadata"](AppRoutingModule, [{
        type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["NgModule"],
        args: [{
                imports: [_angular_router__WEBPACK_IMPORTED_MODULE_1__["RouterModule"].forRoot(routes, { useHash: true })],
                exports: [_angular_router__WEBPACK_IMPORTED_MODULE_1__["RouterModule"]]
            }]
    }], null, null); })();


/***/ }),

/***/ "xD4D":
/*!********************************************!*\
  !*** ./src/app/editor/editor.component.ts ***!
  \********************************************/
/*! exports provided: EditorComponent */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "EditorComponent", function() { return EditorComponent; });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ "fXoL");
/* harmony import */ var three__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! three */ "Womt");
/* harmony import */ var three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! three/examples/jsm/controls/OrbitControls.js */ "RyHr");
/* harmony import */ var dat_gui__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! dat.gui */ "iZKT");
/* harmony import */ var _services_data_service__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ../services/data.service */ "EnSQ");
/* harmony import */ var _services_coordinates_service__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ../services/coordinates.service */ "Hxsc");
/* harmony import */ var _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! @angular/material/grid-list */ "zkoq");
/* harmony import */ var _angular_material_card__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! @angular/material/card */ "Wp6s");
/* harmony import */ var _angular_material_button__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(/*! @angular/material/button */ "bTqV");
/* harmony import */ var _angular_common__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(/*! @angular/common */ "ofXK");
/* harmony import */ var _angular_material_slider__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(/*! @angular/material/slider */ "5RNC");
/* harmony import */ var _angular_forms__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(/*! @angular/forms */ "3Pt+");
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(/*! @angular/material/icon */ "NFeN");
/* harmony import */ var _angular_material_menu__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(/*! @angular/material/menu */ "STbY");
/* harmony import */ var _angular_material_form_field__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(/*! @angular/material/form-field */ "kmnG");
/* harmony import */ var _angular_material_chips__WEBPACK_IMPORTED_MODULE_15__ = __webpack_require__(/*! @angular/material/chips */ "A5z7");

















const _c0 = ["editorContainer"];
function EditorComponent_mat_icon_13_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](1, "play_arrow");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
} }
function EditorComponent_mat_icon_14_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](1, "pause");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
} }
function EditorComponent_mat_card_content_26_mat_chip_20_Template(rf, ctx) { if (rf & 1) {
    const _r11 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵgetCurrentView"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "mat-chip", 18);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("removed", function EditorComponent_mat_card_content_26_mat_chip_20_Template_mat_chip_removed_0_listener() { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵrestoreView"](_r11); const joint_r8 = ctx.$implicit; const i_r5 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵnextContext"]().index; const ctx_r9 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵnextContext"](); return ctx_r9.removeJoint(joint_r8, i_r5); });
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](2, "mat-icon", 19);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](3, "cancel");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
} if (rf & 2) {
    const joint_r8 = ctx.$implicit;
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("removable", true);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtextInterpolate1"]("", joint_r8, " ");
} }
function EditorComponent_mat_card_content_26_Template(rf, ctx) { if (rf & 1) {
    const _r13 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵgetCurrentView"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "mat-card-content");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](1, "h4");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](2);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](3, "button", 13);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](4, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](5, "more_vert");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](6, "mat-menu", null, 14);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](8, "button", 15);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("click", function EditorComponent_mat_card_content_26_Template_button_click_8_listener() { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵrestoreView"](_r13); const i_r5 = ctx.index; const ctx_r12 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵnextContext"](); return ctx_r12.deleteStage(i_r5); });
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](9, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](10, "delete");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](11, "span");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](12, "Delete");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](13, "button", 15);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("click", function EditorComponent_mat_card_content_26_Template_button_click_13_listener() { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵrestoreView"](_r13); const i_r5 = ctx.index; const ctx_r14 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵnextContext"](); return ctx_r14.showStage(i_r5); });
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](14, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](15, "preview");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](16, "span");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](17, "Show Stage");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](18, "mat-form-field", 16);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](19, "mat-chip-list");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtemplate"](20, EditorComponent_mat_card_content_26_mat_chip_20_Template, 4, 2, "mat-chip", 17);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
} if (rf & 2) {
    const stage_r4 = ctx.$implicit;
    const i_r5 = ctx.index;
    const _r6 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵreference"](7);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](2);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtextInterpolate1"]("Stage ", i_r5 + 1, " ");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("matMenuTriggerFor", _r6);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](17);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngForOf", stage_r4[1]);
} }
class EditorComponent {
    constructor(dataService, coordinatesService) {
        this.dataService = dataService;
        this.coordinatesService = coordinatesService;
        this.raycaster = new three__WEBPACK_IMPORTED_MODULE_1__["Raycaster"]();
        // properties
        this.viewHeight = window.innerHeight - 64;
        this.viewWidth = window.innerWidth;
        this.pixelRatio = window.devicePixelRatio;
        this.mouse = new three__WEBPACK_IMPORTED_MODULE_1__["Vector2"]();
        this.touch = new three__WEBPACK_IMPORTED_MODULE_1__["Vector2"]();
        // skeleton
        this.dots = [];
        this.dotsMapping = [];
        this.lines = [];
        this.exName = "Bsp. squats";
        this.playbackSpeed = 1;
        this.playPosition = 1;
        this.isPlaying = false;
        this.max = 100;
        this.selectedJoint = "";
        this.stages = [];
        const width = window.innerWidth;
        const height = window.innerHeight;
        this.renderer = new three__WEBPACK_IMPORTED_MODULE_1__["WebGLRenderer"]({ antialias: true });
        this.scene = new three__WEBPACK_IMPORTED_MODULE_1__["Scene"]();
        this.camera = new three__WEBPACK_IMPORTED_MODULE_1__["PerspectiveCamera"](25, (width * 2 / 3) / height, 0.1, 1000);
        this.orbitControls = new three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_2__["OrbitControls"](this.camera, this.renderer.domElement);
        this.orbitControls.update();
        this.gui = new dat_gui__WEBPACK_IMPORTED_MODULE_3__["GUI"]({ autoPlace: false });
    }
    // event listeners
    onWindowResize(event) {
        const width = window.innerWidth;
        const height = window.innerHeight - 64;
        this.renderer.setSize(width * 2 / 3, height);
        this.camera.aspect = (width * 2 / 3) / height;
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
    ngAfterViewInit() {
        this.camera.lookAt(this.scene.position);
        this.editorContainer.nativeElement.appendChild(this.renderer.domElement);
        this.editorContainer.nativeElement.addEventListener('click', (event) => { this.addJointToStage(this.selectedJoint); });
        this.renderer.setPixelRatio(this.pixelRatio);
        this.renderer.setSize(this.viewWidth * 2 / 3, this.viewHeight);
        this.animate();
    }
    animate() {
        window.requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
        this.selectJoint();
    }
    ngOnInit() {
        var _a;
        this.playbackSpeed = this.coordinatesService.playbackSpeed;
        (_a = document.getElementById('menuContainer')) === null || _a === void 0 ? void 0 : _a.appendChild(this.gui.domElement);
        this.initMenu();
        this.initScene();
    }
    initScene() {
        this.renderer.setClearColor(0xffffff);
        this.camera.position.set(20, 20, 20);
        this.orbitControls.enablePan = true;
        this.orbitControls.enableZoom = true;
        this.orbitControls.enableKeys = true;
        const axesHelper = new three__WEBPACK_IMPORTED_MODULE_1__["AxesHelper"](5);
        this.scene.add(axesHelper);
        const ambientLight = new three__WEBPACK_IMPORTED_MODULE_1__["AmbientLight"](0xffffff);
        ambientLight.intensity = 2;
        this.scene.add(ambientLight);
        this.initSkeleton();
    }
    initMenu() {
        const menu = {
            play: () => { console.log('playing'); this.coordinatesService.playRecording(); },
            check: () => { console.log(this.coordinatesService.recording); }
        };
        const play = this.gui.add(menu, 'play').name('Play');
        const check = this.gui.add(menu, 'check').name('🤔');
    }
    initSkeleton() {
        // tslint:disable-next-line: forin
        for (const point in this.coordinatesService.lastPose) {
            const geometry = new three__WEBPACK_IMPORTED_MODULE_1__["SphereBufferGeometry"](0.1, 32, 32);
            const material = new three__WEBPACK_IMPORTED_MODULE_1__["MeshBasicMaterial"]({ color: 0xafaab9 });
            const dot = new three__WEBPACK_IMPORTED_MODULE_1__["Mesh"](geometry, material);
            dot.name = point.toString();
            this.dots.push({ index: point, dot });
            this.scene.add(dot);
        }
        // tslint:disable-next-line: forin
        for (const start in this.coordinatesService.connections) {
            // tslint:disable-next-line: forin
            for (const end of this.coordinatesService.connections[start]) {
                const material = new three__WEBPACK_IMPORTED_MODULE_1__["LineBasicMaterial"]({
                    color: 0x0000ff,
                    linewidth: 4.0
                });
                const geometry = new three__WEBPACK_IMPORTED_MODULE_1__["BufferGeometry"]();
                const line = new three__WEBPACK_IMPORTED_MODULE_1__["Line"](geometry, material);
                this.lines.push(line);
                this.dotsMapping.push([start, end]);
                this.scene.add(line);
            }
        }
        this.coordinatesService.play.subscribe(pose => {
            if (pose) {
                this.recordingPose = pose;
                this.renderSkeleton(5, 0, 0, 0);
            }
        });
        this.coordinatesService.playPosition.subscribe(pos => {
            this.playPosition = pos;
            this.max = this.coordinatesService.recording.length - 1;
        });
    }
    renderSkeleton(scalar, offsetX, offsetY, offsetZ) {
        const pose = this.recordingPose;
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
            const start = new three__WEBPACK_IMPORTED_MODULE_1__["Vector3"](x0, y0, z0);
            const end = new three__WEBPACK_IMPORTED_MODULE_1__["Vector3"](x1, y1, z1);
            line.geometry.setFromPoints([start, end]);
            line.geometry.computeBoundingBox();
            line.geometry.computeBoundingSphere();
        }
    }
    play() {
        this.coordinatesService.playRecording();
    }
    stop() {
        this.coordinatesService.stopPlayingRecording();
    }
    playPause() {
        if (!this.isPlaying) {
            this.coordinatesService.playRecording();
            this.isPlaying = true;
        }
        else {
            this.coordinatesService.stopPlayingRecording();
            this.isPlaying = false;
        }
    }
    save() {
        this.coordinatesService.saveRecording();
    }
    setSpeed($event) {
        this.coordinatesService.setPlaybackSpeed($event);
    }
    setPlayPosition($event) {
        this.coordinatesService.setPlayPosition(Math.round($event.value));
    }
    selectJoint() {
        this.raycaster.setFromCamera(this.mouse, this.camera);
        const mouseIntersection = this.raycaster.intersectObjects(this.scene.children);
        let go = "";
        for (const intersected of mouseIntersection) {
            if (intersected.object instanceof three__WEBPACK_IMPORTED_MODULE_1__["Mesh"]) {
                go = intersected.object.name;
            }
        }
        if (go.length >= 1) {
            this.selectedJoint = go;
        }
        else {
            this.selectedJoint = "";
        }
    }
    addToStages() {
        const pose = this.coordinatesService.recording[this.coordinatesService.ri][1];
        this.stages.push([pose, new Set()]);
    }
    addJointToStage(joint) {
        if (joint.length > 0) {
            this.stages[this.stages.length - 1][1].add(joint);
        }
    }
    removeJoint(joint, index) {
        this.stages[index][0].remove(joint);
    }
    deleteStage(index) {
        this.stages.splice(index, 1);
    }
    showStage(index) {
        this.coordinatesService.stopPlayingRecording();
        this.coordinatesService.play.next(this.stages[index][0]);
    }
}
EditorComponent.ɵfac = function EditorComponent_Factory(t) { return new (t || EditorComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdirectiveInject"](_services_data_service__WEBPACK_IMPORTED_MODULE_4__["DataService"]), _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdirectiveInject"](_services_coordinates_service__WEBPACK_IMPORTED_MODULE_5__["CoordinatesService"])); };
EditorComponent.ɵcmp = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineComponent"]({ type: EditorComponent, selectors: [["app-editor"]], viewQuery: function EditorComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵviewQuery"](_c0, true);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵloadQuery"]()) && (ctx.editorContainer = _t.first);
    } }, hostBindings: function EditorComponent_HostBindings(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("resize", function EditorComponent_resize_HostBindingHandler($event) { return ctx.onWindowResize($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵresolveWindow"])("mousemove", function EditorComponent_mousemove_HostBindingHandler($event) { return ctx.onDocumentMouseMove($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵresolveDocument"])("touchend", function EditorComponent_touchend_HostBindingHandler($event) { return ctx.onTouchEnd($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵresolveDocument"]);
    } }, decls: 27, vars: 15, consts: [["cols", "3", 3, "rowHeight"], ["id", "rc", 3, "colspan", "rowspan"], ["id", "editorContainer", 1, "div-editor"], ["editorContainer", ""], [3, "colspan", "rowspan"], [1, "exercise-card"], ["mat-mini-fab", "", "color", "primary", 3, "click"], [4, "ngIf"], [1, "play-slider", 3, "ngModel", "min", "max", "ngModelChange", "input"], [1, "speed-span", "toolbar-spacer"], [3, "ngModel", "max", "min", "step", "ngModelChange"], ["mat-button", "", "color", "primary", 3, "click"], [4, "ngFor", "ngForOf"], ["mat-icon-button", "", "aria-label", "Example icon-button with a menu", 3, "matMenuTriggerFor"], ["menu", "matMenu"], ["mat-menu-item", "", 3, "click"], [1, "stage-joints"], [3, "removable", "removed", 4, "ngFor", "ngForOf"], [3, "removable", "removed"], ["matChipRemove", ""]], template: function EditorComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "mat-grid-list", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](1, "mat-grid-tile", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelement"](2, "div", 2, 3);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](4, "mat-grid-tile", 4);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](5, "mat-card", 5);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](6, "mat-card-header");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](7, "mat-card-title");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](8, "Recording Editor");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](9, "mat-card-subtitle");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](10, "Stage defining");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](11, "mat-card-content");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](12, "button", 6);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_12_listener() { return ctx.playPause(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtemplate"](13, EditorComponent_mat_icon_13_Template, 2, 0, "mat-icon", 7);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtemplate"](14, EditorComponent_mat_icon_14_Template, 2, 0, "mat-icon", 7);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](15, "mat-slider", 8);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("ngModelChange", function EditorComponent_Template_mat_slider_ngModelChange_15_listener($event) { return ctx.playPosition = $event; })("input", function EditorComponent_Template_mat_slider_input_15_listener($event) { return ctx.setPlayPosition($event); });
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](16, "mat-card-content");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](17, "span", 9);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](18, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](19, "speed");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](20, "mat-slider", 10);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("ngModelChange", function EditorComponent_Template_mat_slider_ngModelChange_20_listener($event) { return ctx.setSpeed($event); });
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](21, "mat-card-content");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](22, "button", 11);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_22_listener() { return ctx.addToStages(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](23, "Add Stage");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](24, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](25, "add");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtemplate"](26, EditorComponent_mat_card_content_26_Template, 21, 3, "mat-card-content", 12);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    } if (rf & 2) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("rowHeight", ctx.viewHeight / 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("colspan", 2)("rowspan", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](3);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("colspan", 1)("rowspan", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](9);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngIf", !ctx.isPlaying);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngIf", ctx.isPlaying);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngModel", ctx.playPosition)("min", 1)("max", ctx.max);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](5);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngModel", ctx.playbackSpeed)("max", 2)("min", 0.1)("step", 0.01);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](6);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngForOf", ctx.stages);
    } }, directives: [_angular_material_grid_list__WEBPACK_IMPORTED_MODULE_6__["MatGridList"], _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_6__["MatGridTile"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCard"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardHeader"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardTitle"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardSubtitle"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardContent"], _angular_material_button__WEBPACK_IMPORTED_MODULE_8__["MatButton"], _angular_common__WEBPACK_IMPORTED_MODULE_9__["NgIf"], _angular_material_slider__WEBPACK_IMPORTED_MODULE_10__["MatSlider"], _angular_forms__WEBPACK_IMPORTED_MODULE_11__["NgControlStatus"], _angular_forms__WEBPACK_IMPORTED_MODULE_11__["NgModel"], _angular_material_icon__WEBPACK_IMPORTED_MODULE_12__["MatIcon"], _angular_common__WEBPACK_IMPORTED_MODULE_9__["NgForOf"], _angular_material_menu__WEBPACK_IMPORTED_MODULE_13__["MatMenuTrigger"], _angular_material_menu__WEBPACK_IMPORTED_MODULE_13__["MatMenu"], _angular_material_menu__WEBPACK_IMPORTED_MODULE_13__["MatMenuItem"], _angular_material_form_field__WEBPACK_IMPORTED_MODULE_14__["MatFormField"], _angular_material_chips__WEBPACK_IMPORTED_MODULE_15__["MatChipList"], _angular_material_chips__WEBPACK_IMPORTED_MODULE_15__["MatChip"], _angular_material_chips__WEBPACK_IMPORTED_MODULE_15__["MatChipRemove"]], styles: [".div-editor[_ngcontent-%COMP%] {\n  position: relative;\n  display: flex;\n  justify-content: center;\n}\n\n#menuContainer[_ngcontent-%COMP%] {\n  position: relative;\n  top: 0em;\n  right: 0em;\n  z-index: 2;\n}\n\n.mat-icon[_ngcontent-%COMP%] {\n  vertical-align: middle;\n}\n\n.speed-span[_ngcontent-%COMP%] {\n  margin-left: 0px;\n}\n\n.play-slider[_ngcontent-%COMP%] {\n  margin-left: 8px;\n  width: 90%;\n}\n\nmat-card[_ngcontent-%COMP%] {\n  width: 100%;\n}\n\n.stage-joints[_ngcontent-%COMP%] {\n  width: 100%;\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIi4uLy4uLy4uL2VkaXRvci5jb21wb25lbnQuc2NzcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtFQUNJLGtCQUFBO0VBQ0EsYUFBQTtFQUNBLHVCQUFBO0FBQ0o7O0FBRUU7RUFDRSxrQkFBQTtFQUNBLFFBQUE7RUFDQSxVQUFBO0VBQ0EsVUFBQTtBQUNKOztBQUdFO0VBQ0Usc0JBQUE7QUFBSjs7QUFHQTtFQUNFLGdCQUFBO0FBQUY7O0FBR0E7RUFDRSxnQkFBQTtFQUNBLFVBQUE7QUFBRjs7QUFHQTtFQUNFLFdBQUE7QUFBRjs7QUFFQTtFQUNFLFdBQUE7QUFDRiIsImZpbGUiOiJlZGl0b3IuY29tcG9uZW50LnNjc3MiLCJzb3VyY2VzQ29udGVudCI6WyIuZGl2LWVkaXRvciB7XG4gICAgcG9zaXRpb246IHJlbGF0aXZlO1xuICAgIGRpc3BsYXk6IGZsZXg7XG4gICAganVzdGlmeS1jb250ZW50OiBjZW50ZXI7XG4gIH1cbiAgXG4gICNtZW51Q29udGFpbmVyIHtcbiAgICBwb3NpdGlvbjogcmVsYXRpdmU7XG4gICAgdG9wOiAwZW07XG4gICAgcmlnaHQ6IDBlbTtcbiAgICB6LWluZGV4OiAyO1xuICB9XG4gIFxuXG4gIC5tYXQtaWNvbiB7XG4gICAgdmVydGljYWwtYWxpZ246IG1pZGRsZTtcbn1cblxuLnNwZWVkLXNwYW4ge1xuICBtYXJnaW4tbGVmdDogMHB4O1xufVxuXG4ucGxheS1zbGlkZXIge1xuICBtYXJnaW4tbGVmdDogOHB4O1xuICB3aWR0aDogOTAlO1xufVxuICBcbm1hdC1jYXJkIHtcbiAgd2lkdGg6IDEwMCU7XG59XG4uc3RhZ2Utam9pbnRzIHtcbiAgd2lkdGg6IDEwMCU7XG59Il19 */"] });
/*@__PURE__*/ (function () { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵsetClassMetadata"](EditorComponent, [{
        type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["Component"],
        args: [{
                selector: 'app-editor',
                templateUrl: './editor.component.html',
                styleUrls: ['./editor.component.scss']
            }]
    }], function () { return [{ type: _services_data_service__WEBPACK_IMPORTED_MODULE_4__["DataService"] }, { type: _services_coordinates_service__WEBPACK_IMPORTED_MODULE_5__["CoordinatesService"] }]; }, { onWindowResize: [{
            type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["HostListener"],
            args: ['window:resize', ['$event']]
        }], onDocumentMouseMove: [{
            type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["HostListener"],
            args: ['document:mousemove', ['$event']]
        }], onTouchEnd: [{
            type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["HostListener"],
            args: ['document:touchend', ['$event']]
        }], editorContainer: [{
            type: _angular_core__WEBPACK_IMPORTED_MODULE_0__["ViewChild"],
            args: ['editorContainer']
        }] }); })();


/***/ }),

/***/ "zUnb":
/*!*********************!*\
  !*** ./src/main.ts ***!
  \*********************/
/*! no exports provided */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ "fXoL");
/* harmony import */ var _environments_environment__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./environments/environment */ "AytR");
/* harmony import */ var _app_app_module__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./app/app.module */ "ZAI4");
/* harmony import */ var _angular_platform_browser__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/platform-browser */ "jhN1");




if (_environments_environment__WEBPACK_IMPORTED_MODULE_1__["environment"].production) {
    Object(_angular_core__WEBPACK_IMPORTED_MODULE_0__["enableProdMode"])();
}
_angular_platform_browser__WEBPACK_IMPORTED_MODULE_3__["platformBrowser"]().bootstrapModule(_app_app_module__WEBPACK_IMPORTED_MODULE_2__["AppModule"])
    .catch(err => console.error(err));


/***/ }),

/***/ "zn8P":
/*!******************************************************!*\
  !*** ./$$_lazy_route_resource lazy namespace object ***!
  \******************************************************/
/*! no static exports found */
/***/ (function(module, exports) {

function webpackEmptyAsyncContext(req) {
	// Here Promise.resolve().then() is used instead of new Promise() to prevent
	// uncaught exception popping up in devtools
	return Promise.resolve().then(function() {
		var e = new Error("Cannot find module '" + req + "'");
		e.code = 'MODULE_NOT_FOUND';
		throw e;
	});
}
webpackEmptyAsyncContext.keys = function() { return []; };
webpackEmptyAsyncContext.resolve = webpackEmptyAsyncContext;
module.exports = webpackEmptyAsyncContext;
webpackEmptyAsyncContext.id = "zn8P";

/***/ })

},[[0,"runtime","vendor"]]]);
//# sourceMappingURL=main.js.map