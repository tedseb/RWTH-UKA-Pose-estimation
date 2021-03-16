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
        this.socket = dataService.getWebsocket();
        this.socket.subscribe(message => {
            this.lastPose = message;
            this.update.next(this.lastPose);
        });
        this.update = new rxjs__WEBPACK_IMPORTED_MODULE_1__["BehaviorSubject"](this.lastPose);
        this.play = new rxjs__WEBPACK_IMPORTED_MODULE_1__["BehaviorSubject"](this.lastPose);
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
        console.log('being called');
        if (this.ri === this.recording.length - 1) {
            this.ri = 1;
        }
        let time = this.recording[this.ri][0] - this.recording[this.ri - 1][0];
        if (this.recording) {
            setTimeout(() => {
                this.play.next(this.recording[this.ri][1]);
                this.ri++;
                if (this.isPlaying === true) {
                    this.playingRecording();
                }
            }, time * (1 / this.playbackSpeed));
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
            _angular_material_slider__WEBPACK_IMPORTED_MODULE_20__["MatSliderModule"]
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
        _angular_material_slider__WEBPACK_IMPORTED_MODULE_20__["MatSliderModule"]] }); })();
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
                    _angular_material_slider__WEBPACK_IMPORTED_MODULE_20__["MatSliderModule"]
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
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(/*! @angular/material/icon */ "NFeN");
/* harmony import */ var _angular_material_slider__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(/*! @angular/material/slider */ "5RNC");
/* harmony import */ var _angular_forms__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(/*! @angular/forms */ "3Pt+");
/* harmony import */ var _angular_material_form_field__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(/*! @angular/material/form-field */ "kmnG");
/* harmony import */ var _angular_material_input__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(/*! @angular/material/input */ "qFsG");
/* harmony import */ var _angular_common__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(/*! @angular/common */ "ofXK");
















const _c0 = ["editorContainer"];
function EditorComponent_button_27_Template(rf, ctx) { if (rf & 1) {
    const _r3 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵgetCurrentView"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "button", 11);
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("click", function EditorComponent_button_27_Template_button_click_0_listener() { _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵrestoreView"](_r3); const ctx_r2 = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵnextContext"](); return ctx_r2.exName = ""; });
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](1, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](2, "close");
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
} }
class EditorComponent {
    constructor(dataService, coordinatesService) {
        this.dataService = dataService;
        this.coordinatesService = coordinatesService;
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
    ngAfterViewInit() {
        this.camera.lookAt(this.scene.position);
        this.editorContainer.nativeElement.appendChild(this.renderer.domElement);
        this.renderer.setPixelRatio(this.pixelRatio);
        this.renderer.setSize(this.viewWidth * 2 / 3, this.viewHeight);
        this.animate();
    }
    animate() {
        window.requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }
    ngOnInit() {
        var _a;
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
    save() {
        this.coordinatesService.saveRecording();
    }
    setSpeed($event) {
        this.coordinatesService.setPlaybackSpeed($event);
    }
}
EditorComponent.ɵfac = function EditorComponent_Factory(t) { return new (t || EditorComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdirectiveInject"](_services_data_service__WEBPACK_IMPORTED_MODULE_4__["DataService"]), _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdirectiveInject"](_services_coordinates_service__WEBPACK_IMPORTED_MODULE_5__["CoordinatesService"])); };
EditorComponent.ɵcmp = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineComponent"]({ type: EditorComponent, selectors: [["app-editor"]], viewQuery: function EditorComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵviewQuery"](_c0, true);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵloadQuery"]()) && (ctx.editorContainer = _t.first);
    } }, hostBindings: function EditorComponent_HostBindings(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("resize", function EditorComponent_resize_HostBindingHandler($event) { return ctx.onWindowResize($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵresolveWindow"]);
    } }, decls: 29, vars: 13, consts: [["cols", "3", 3, "rowHeight"], ["id", "rc", 3, "colspan", "rowspan"], ["id", "editorContainer", 1, "div-editor"], ["editorContainer", ""], [3, "colspan", "rowspan"], [1, "exercise-card"], ["mat-button", "", 3, "click"], [3, "ngModel", "max", "min", "step", "ngModelChange"], [1, "example-form-field"], ["matInput", "", "type", "text", 3, "ngModel", "ngModelChange"], ["mat-button", "", "matSuffix", "", "mat-icon-button", "", "aria-label", "Clear", 3, "click", 4, "ngIf"], ["mat-button", "", "matSuffix", "", "mat-icon-button", "", "aria-label", "Clear", 3, "click"]], template: function EditorComponent_Template(rf, ctx) { if (rf & 1) {
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
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](12, "p");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](13, "button", 6);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_13_listener() { return ctx.play(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](14, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](15, "play_arrow");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](16, "button", 6);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_16_listener() { return ctx.stop(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](17, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](18, "pause");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](19, "mat-slider", 7);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("ngModelChange", function EditorComponent_Template_mat_slider_ngModelChange_19_listener($event) { return ctx.setSpeed($event); });
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](20, "mat-card-actions");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](21, "p");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](22, "Name");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](23, "mat-form-field", 8);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](24, "mat-label");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](25, "\u00DCbungsname");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](26, "input", 9);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵlistener"]("ngModelChange", function EditorComponent_Template_input_ngModelChange_26_listener($event) { return ctx.exName = $event; });
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtemplate"](27, EditorComponent_button_27_Template, 3, 0, "button", 10);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelement"](28, "mat-grid-tile", 4);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
    } if (rf & 2) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("rowHeight", ctx.viewHeight / 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("colspan", 2)("rowspan", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](3);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("colspan", 1)("rowspan", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](15);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngModel", ctx.playbackSpeed)("max", 2)("min", 0)("step", 0.01);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](7);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngModel", ctx.exName);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("ngIf", ctx.exName);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵproperty"]("colspan", 1)("rowspan", 1);
    } }, directives: [_angular_material_grid_list__WEBPACK_IMPORTED_MODULE_6__["MatGridList"], _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_6__["MatGridTile"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCard"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardHeader"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardTitle"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardSubtitle"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardContent"], _angular_material_button__WEBPACK_IMPORTED_MODULE_8__["MatButton"], _angular_material_icon__WEBPACK_IMPORTED_MODULE_9__["MatIcon"], _angular_material_slider__WEBPACK_IMPORTED_MODULE_10__["MatSlider"], _angular_forms__WEBPACK_IMPORTED_MODULE_11__["NgControlStatus"], _angular_forms__WEBPACK_IMPORTED_MODULE_11__["NgModel"], _angular_material_card__WEBPACK_IMPORTED_MODULE_7__["MatCardActions"], _angular_material_form_field__WEBPACK_IMPORTED_MODULE_12__["MatFormField"], _angular_material_form_field__WEBPACK_IMPORTED_MODULE_12__["MatLabel"], _angular_material_input__WEBPACK_IMPORTED_MODULE_13__["MatInput"], _angular_forms__WEBPACK_IMPORTED_MODULE_11__["DefaultValueAccessor"], _angular_common__WEBPACK_IMPORTED_MODULE_14__["NgIf"], _angular_material_form_field__WEBPACK_IMPORTED_MODULE_12__["MatSuffix"]], styles: [".div-editor[_ngcontent-%COMP%] {\n  position: relative;\n  display: flex;\n  justify-content: center;\n}\n\n#menuContainer[_ngcontent-%COMP%] {\n  position: relative;\n  top: 0em;\n  right: 0em;\n  z-index: 2;\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIi4uLy4uLy4uL2VkaXRvci5jb21wb25lbnQuc2NzcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtFQUNJLGtCQUFBO0VBQ0EsYUFBQTtFQUNBLHVCQUFBO0FBQ0o7O0FBRUU7RUFDRSxrQkFBQTtFQUNBLFFBQUE7RUFDQSxVQUFBO0VBQ0EsVUFBQTtBQUNKIiwiZmlsZSI6ImVkaXRvci5jb21wb25lbnQuc2NzcyIsInNvdXJjZXNDb250ZW50IjpbIi5kaXYtZWRpdG9yIHtcbiAgICBwb3NpdGlvbjogcmVsYXRpdmU7XG4gICAgZGlzcGxheTogZmxleDtcbiAgICBqdXN0aWZ5LWNvbnRlbnQ6IGNlbnRlcjtcbiAgfVxuICBcbiAgI21lbnVDb250YWluZXIge1xuICAgIHBvc2l0aW9uOiByZWxhdGl2ZTtcbiAgICB0b3A6IDBlbTtcbiAgICByaWdodDogMGVtO1xuICAgIHotaW5kZXg6IDI7XG4gIH1cbiAgIl19 */"] });
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