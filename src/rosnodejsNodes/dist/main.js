(self["webpackChunkhmi"] = self["webpackChunkhmi"] || []).push([["main"],{

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
/* harmony import */ var _angular_router__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! @angular/router */ 9895);
/* harmony import */ var _edk_editor_editor_component__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./edk/editor/editor.component */ 142);
/* harmony import */ var _edk_edk_component__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./edk/edk.component */ 8802);
/* harmony import */ var _edk_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./edk/renderer/renderer.component */ 6632);
/* harmony import */ var _edk_labeler_labeler_component__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./edk/labeler/labeler.component */ 881);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! @angular/core */ 7716);







const routes = [
    { path: 'edk', component: _edk_edk_component__WEBPACK_IMPORTED_MODULE_1__.EDKComponent },
    { path: 'labeler', component: _edk_labeler_labeler_component__WEBPACK_IMPORTED_MODULE_3__.LabelerComponent },
    { path: 'editor', component: _edk_editor_editor_component__WEBPACK_IMPORTED_MODULE_0__.EditorComponent },
    { path: 'renderer', component: _edk_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_2__.RendererComponent },
    { path: '', redirectTo: '/renderer', pathMatch: 'full' }
];
class AppRoutingModule {
}
AppRoutingModule.ɵfac = function AppRoutingModule_Factory(t) { return new (t || AppRoutingModule)(); };
AppRoutingModule.ɵmod = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵdefineNgModule"]({ type: AppRoutingModule });
AppRoutingModule.ɵinj = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵdefineInjector"]({ imports: [[_angular_router__WEBPACK_IMPORTED_MODULE_5__.RouterModule.forRoot(routes, { useHash: true })], _angular_router__WEBPACK_IMPORTED_MODULE_5__.RouterModule] });
(function () { (typeof ngJitMode === "undefined" || ngJitMode) && _angular_core__WEBPACK_IMPORTED_MODULE_4__["ɵɵsetNgModuleScope"](AppRoutingModule, { imports: [_angular_router__WEBPACK_IMPORTED_MODULE_5__.RouterModule], exports: [_angular_router__WEBPACK_IMPORTED_MODULE_5__.RouterModule] }); })();


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
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! @angular/core */ 7716);
/* harmony import */ var _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/material/toolbar */ 2522);
/* harmony import */ var _angular_material_button__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/material/button */ 1095);
/* harmony import */ var _angular_router__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/router */ 9895);
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! @angular/material/icon */ 6627);





class AppComponent {
    constructor() {
        this.title = 'hmi';
    }
}
AppComponent.ɵfac = function AppComponent_Factory(t) { return new (t || AppComponent)(); };
AppComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵdefineComponent"]({ type: AppComponent, selectors: [["app-root"]], decls: 18, vars: 0, consts: [["id", "toolbar-paragraph"], ["mat-button", "", "color", "primary", "routerLink", "/renderer"], ["mat-button", "", "color", "primary", "routerLink", "/edk"], ["mat-button", "", "color", "primary", "routerLink", "/editor"], [1, "toolbar-spacer"], ["mat-icon-button", "", "color", "primary", "aria-label", "Heart icon"], ["mat-icon-button", "", "color", "primary", "aria-label", "Share icon"]], template: function AppComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](0, "p", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](1, "mat-toolbar");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](2, "span");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](3, "TrainerAI");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](4, "button", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](5, "Viewer");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](6, "button", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](7, "EDK");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](8, "button", 3);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](9, "Analytics");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelement"](10, "span", 4);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](11, "button", 5);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](12, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](13, "favorite");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](14, "button", 6);
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementStart"](15, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵtext"](16, "share");
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_0__["ɵɵelement"](17, "router-outlet");
    } }, directives: [_angular_material_toolbar__WEBPACK_IMPORTED_MODULE_1__.MatToolbar, _angular_material_button__WEBPACK_IMPORTED_MODULE_2__.MatButton, _angular_router__WEBPACK_IMPORTED_MODULE_3__.RouterLink, _angular_material_icon__WEBPACK_IMPORTED_MODULE_4__.MatIcon, _angular_router__WEBPACK_IMPORTED_MODULE_3__.RouterOutlet], styles: ["\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IiIsImZpbGUiOiJhcHAuY29tcG9uZW50LnNjc3MifQ== */"] });


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
/* harmony import */ var _angular_platform_browser__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(/*! @angular/platform-browser */ 9075);
/* harmony import */ var _angular_common_http__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(/*! @angular/common/http */ 1841);
/* harmony import */ var _app_routing_module__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./app-routing.module */ 158);
/* harmony import */ var _app_component__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./app.component */ 5041);
/* harmony import */ var _edk_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./edk/renderer/renderer.component */ 6632);
/* harmony import */ var _edk_edk_component__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./edk/edk.component */ 8802);
/* harmony import */ var _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(/*! @angular/platform-browser/animations */ 5835);
/* harmony import */ var _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(/*! @angular/material/toolbar */ 2522);
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(/*! @angular/material/icon */ 6627);
/* harmony import */ var _angular_material_button__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(/*! @angular/material/button */ 1095);
/* harmony import */ var _angular_material_sidenav__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(/*! @angular/material/sidenav */ 4935);
/* harmony import */ var _angular_material_slide_toggle__WEBPACK_IMPORTED_MODULE_17__ = __webpack_require__(/*! @angular/material/slide-toggle */ 5396);
/* harmony import */ var _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_15__ = __webpack_require__(/*! @angular/material/grid-list */ 4929);
/* harmony import */ var _angular_material_badge__WEBPACK_IMPORTED_MODULE_19__ = __webpack_require__(/*! @angular/material/badge */ 346);
/* harmony import */ var _angular_material_select__WEBPACK_IMPORTED_MODULE_18__ = __webpack_require__(/*! @angular/material/select */ 7441);
/* harmony import */ var _angular_forms__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! @angular/forms */ 3679);
/* harmony import */ var _angular_material_card__WEBPACK_IMPORTED_MODULE_16__ = __webpack_require__(/*! @angular/material/card */ 3738);
/* harmony import */ var _edk_editor_editor_component__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./edk/editor/editor.component */ 142);
/* harmony import */ var _angular_material_form_field__WEBPACK_IMPORTED_MODULE_20__ = __webpack_require__(/*! @angular/material/form-field */ 8295);
/* harmony import */ var _angular_material_input__WEBPACK_IMPORTED_MODULE_21__ = __webpack_require__(/*! @angular/material/input */ 3166);
/* harmony import */ var _angular_material_slider__WEBPACK_IMPORTED_MODULE_22__ = __webpack_require__(/*! @angular/material/slider */ 4436);
/* harmony import */ var _angular_material_menu__WEBPACK_IMPORTED_MODULE_24__ = __webpack_require__(/*! @angular/material/menu */ 3935);
/* harmony import */ var _angular_material_chips__WEBPACK_IMPORTED_MODULE_23__ = __webpack_require__(/*! @angular/material/chips */ 8341);
/* harmony import */ var _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_25__ = __webpack_require__(/*! @angular/material/snack-bar */ 7001);
/* harmony import */ var _angular_material_dialog__WEBPACK_IMPORTED_MODULE_26__ = __webpack_require__(/*! @angular/material/dialog */ 2238);
/* harmony import */ var _angular_material_table__WEBPACK_IMPORTED_MODULE_27__ = __webpack_require__(/*! @angular/material/table */ 2091);
/* harmony import */ var _angular_material_tabs__WEBPACK_IMPORTED_MODULE_28__ = __webpack_require__(/*! @angular/material/tabs */ 5939);
/* harmony import */ var _edk_labeler_labeler_component__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ./edk/labeler/labeler.component */ 881);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! @angular/core */ 7716);


// component imports




// angular material module imports























class AppModule {
}
AppModule.ɵfac = function AppModule_Factory(t) { return new (t || AppModule)(); };
AppModule.ɵmod = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_6__["ɵɵdefineNgModule"]({ type: AppModule, bootstrap: [_app_component__WEBPACK_IMPORTED_MODULE_1__.AppComponent] });
AppModule.ɵinj = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_6__["ɵɵdefineInjector"]({ providers: [], imports: [[
            _angular_forms__WEBPACK_IMPORTED_MODULE_7__.FormsModule,
            _angular_platform_browser__WEBPACK_IMPORTED_MODULE_8__.BrowserModule,
            _app_routing_module__WEBPACK_IMPORTED_MODULE_0__.AppRoutingModule,
            _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_9__.BrowserAnimationsModule,
            _angular_common_http__WEBPACK_IMPORTED_MODULE_10__.HttpClientModule,
            _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_11__.MatToolbarModule,
            _angular_material_icon__WEBPACK_IMPORTED_MODULE_12__.MatIconModule,
            _angular_material_button__WEBPACK_IMPORTED_MODULE_13__.MatButtonModule,
            _angular_material_sidenav__WEBPACK_IMPORTED_MODULE_14__.MatSidenavModule,
            _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_15__.MatGridListModule,
            _angular_material_card__WEBPACK_IMPORTED_MODULE_16__.MatCardModule,
            _angular_material_slide_toggle__WEBPACK_IMPORTED_MODULE_17__.MatSlideToggleModule,
            _angular_material_select__WEBPACK_IMPORTED_MODULE_18__.MatSelectModule,
            _angular_material_badge__WEBPACK_IMPORTED_MODULE_19__.MatBadgeModule,
            _angular_material_form_field__WEBPACK_IMPORTED_MODULE_20__.MatFormFieldModule,
            _angular_material_input__WEBPACK_IMPORTED_MODULE_21__.MatInputModule,
            _angular_material_slider__WEBPACK_IMPORTED_MODULE_22__.MatSliderModule,
            _angular_material_chips__WEBPACK_IMPORTED_MODULE_23__.MatChipsModule,
            _angular_material_menu__WEBPACK_IMPORTED_MODULE_24__.MatMenuModule,
            _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_25__.MatSnackBarModule,
            _angular_material_dialog__WEBPACK_IMPORTED_MODULE_26__.MatDialogModule,
            _angular_material_table__WEBPACK_IMPORTED_MODULE_27__.MatTableModule,
            _angular_material_tabs__WEBPACK_IMPORTED_MODULE_28__.MatTabsModule
        ]] });
(function () { (typeof ngJitMode === "undefined" || ngJitMode) && _angular_core__WEBPACK_IMPORTED_MODULE_6__["ɵɵsetNgModuleScope"](AppModule, { declarations: [_app_component__WEBPACK_IMPORTED_MODULE_1__.AppComponent,
        _edk_renderer_renderer_component__WEBPACK_IMPORTED_MODULE_2__.RendererComponent,
        _edk_editor_editor_component__WEBPACK_IMPORTED_MODULE_4__.EditorComponent,
        _edk_edk_component__WEBPACK_IMPORTED_MODULE_3__.EDKComponent,
        _edk_labeler_labeler_component__WEBPACK_IMPORTED_MODULE_5__.LabelerComponent], imports: [_angular_forms__WEBPACK_IMPORTED_MODULE_7__.FormsModule,
        _angular_platform_browser__WEBPACK_IMPORTED_MODULE_8__.BrowserModule,
        _app_routing_module__WEBPACK_IMPORTED_MODULE_0__.AppRoutingModule,
        _angular_platform_browser_animations__WEBPACK_IMPORTED_MODULE_9__.BrowserAnimationsModule,
        _angular_common_http__WEBPACK_IMPORTED_MODULE_10__.HttpClientModule,
        _angular_material_toolbar__WEBPACK_IMPORTED_MODULE_11__.MatToolbarModule,
        _angular_material_icon__WEBPACK_IMPORTED_MODULE_12__.MatIconModule,
        _angular_material_button__WEBPACK_IMPORTED_MODULE_13__.MatButtonModule,
        _angular_material_sidenav__WEBPACK_IMPORTED_MODULE_14__.MatSidenavModule,
        _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_15__.MatGridListModule,
        _angular_material_card__WEBPACK_IMPORTED_MODULE_16__.MatCardModule,
        _angular_material_slide_toggle__WEBPACK_IMPORTED_MODULE_17__.MatSlideToggleModule,
        _angular_material_select__WEBPACK_IMPORTED_MODULE_18__.MatSelectModule,
        _angular_material_badge__WEBPACK_IMPORTED_MODULE_19__.MatBadgeModule,
        _angular_material_form_field__WEBPACK_IMPORTED_MODULE_20__.MatFormFieldModule,
        _angular_material_input__WEBPACK_IMPORTED_MODULE_21__.MatInputModule,
        _angular_material_slider__WEBPACK_IMPORTED_MODULE_22__.MatSliderModule,
        _angular_material_chips__WEBPACK_IMPORTED_MODULE_23__.MatChipsModule,
        _angular_material_menu__WEBPACK_IMPORTED_MODULE_24__.MatMenuModule,
        _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_25__.MatSnackBarModule,
        _angular_material_dialog__WEBPACK_IMPORTED_MODULE_26__.MatDialogModule,
        _angular_material_table__WEBPACK_IMPORTED_MODULE_27__.MatTableModule,
        _angular_material_tabs__WEBPACK_IMPORTED_MODULE_28__.MatTabsModule] }); })();


/***/ }),

/***/ 142:
/*!************************************************!*\
  !*** ./src/app/edk/editor/editor.component.ts ***!
  \************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "EditorComponent": () => (/* binding */ EditorComponent)
/* harmony export */ });
/* harmony import */ var three__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! three */ 7758);
/* harmony import */ var three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! three/examples/jsm/controls/OrbitControls.js */ 6887);
/* harmony import */ var dat_gui__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! dat.gui */ 4486);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! @angular/core */ 7716);
/* harmony import */ var _services_data_service__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ../../services/data.service */ 2468);
/* harmony import */ var _services_coordinates_service__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ../../services/coordinates.service */ 8498);
/* harmony import */ var _angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! @angular/material/snack-bar */ 7001);
/* harmony import */ var _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! @angular/material/grid-list */ 4929);
/* harmony import */ var _angular_material_tabs__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(/*! @angular/material/tabs */ 5939);
/* harmony import */ var _angular_material_button__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(/*! @angular/material/button */ 1095);
/* harmony import */ var _angular_material_icon__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(/*! @angular/material/icon */ 6627);
/* harmony import */ var _angular_common__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(/*! @angular/common */ 8583);
/* harmony import */ var _angular_material_slider__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(/*! @angular/material/slider */ 4436);
/* harmony import */ var _angular_forms__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(/*! @angular/forms */ 3679);
/* harmony import */ var _angular_material_menu__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(/*! @angular/material/menu */ 3935);
/* harmony import */ var _angular_material_card__WEBPACK_IMPORTED_MODULE_15__ = __webpack_require__(/*! @angular/material/card */ 3738);
/* harmony import */ var _angular_material_form_field__WEBPACK_IMPORTED_MODULE_16__ = __webpack_require__(/*! @angular/material/form-field */ 8295);
/* harmony import */ var _angular_material_input__WEBPACK_IMPORTED_MODULE_17__ = __webpack_require__(/*! @angular/material/input */ 3166);


















const _c0 = ["editorContainer"];
function EditorComponent_p_13_span_2_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "span");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} if (rf & 2) {
    const joint_r12 = ctx.$implicit;
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate1"](" - ", joint_r12, "");
} }
function EditorComponent_p_13_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "p");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](2, EditorComponent_p_13_span_2_Template, 2, 1, "span", 8);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} if (rf & 2) {
    const angles_r9 = ctx.$implicit;
    const j_r10 = ctx.index;
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate1"](" Angle ", j_r10 + 1, " ");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngForOf", angles_r9);
} }
function EditorComponent_p_21_span_2_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "span");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} if (rf & 2) {
    const joint_r16 = ctx.$implicit;
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate1"](" - ", joint_r16, "");
} }
function EditorComponent_p_21_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "p");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](2, EditorComponent_p_21_span_2_Template, 2, 1, "span", 8);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} if (rf & 2) {
    const angles_r13 = ctx.$implicit;
    const j_r14 = ctx.index;
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate1"](" Angle ", j_r14 + 1, " ");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngForOf", angles_r13);
} }
function EditorComponent_div_31_p_6_span_2_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "span");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} if (rf & 2) {
    const joint_r21 = ctx.$implicit;
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate1"](" - ", joint_r21, "");
} }
function EditorComponent_div_31_p_6_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "p");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](2, EditorComponent_div_31_p_6_span_2_Template, 2, 1, "span", 8);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} if (rf & 2) {
    const angles_r18 = ctx.$implicit;
    const j_r19 = ctx.index;
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate1"](" Angle ", j_r19 + 1, " ");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngForOf", angles_r18);
} }
function EditorComponent_div_31_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "div", 25);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](1, "mat-card", 26);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](2, "mat-card-header");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](3, "mat-card-title");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](4, "Added Features");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](5, "mat-card-content");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](6, EditorComponent_div_31_p_6_Template, 3, 2, "p", 8);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelement"](7, "mat-card-actions");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} if (rf & 2) {
    const ctx_r3 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵnextContext"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](6);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngForOf", ctx_r3.anglesMerged[0]);
} }
function EditorComponent_mat_icon_34_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](1, "play_arrow");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} }
function EditorComponent_mat_icon_35_Template(rf, ctx) { if (rf & 1) {
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "mat-icon");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](1, "pause");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} }
function EditorComponent_span_62_Template(rf, ctx) { if (rf & 1) {
    const _r23 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵgetCurrentView"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "span", 27);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](1, "mat-form-field", 28);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](2, "mat-label");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](3, "\u00DCbungsname");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](4, "input", 29);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("ngModelChange", function EditorComponent_span_62_Template_input_ngModelChange_4_listener($event) { _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵrestoreView"](_r23); const ctx_r22 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵnextContext"](); return ctx_r22.exName = $event; });
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](5, "button", 30);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_span_62_Template_button_click_5_listener() { _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵrestoreView"](_r23); const ctx_r24 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵnextContext"](); return ctx_r24.onSave(); });
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](6, "mat-icon", 31);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](7, "save");
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
} if (rf & 2) {
    const ctx_r7 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵnextContext"]();
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](4);
    _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngModel", ctx_r7.exName);
} }
const _c1 = function (a0, a1) { return { "play-slider-big": a0, "play-slider-small": a1 }; };
class EditorComponent {
    constructor(dataService, coordinatesService, _snackbar) {
        this.dataService = dataService;
        this.coordinatesService = coordinatesService;
        this._snackbar = _snackbar;
        this.raycaster = new three__WEBPACK_IMPORTED_MODULE_0__.Raycaster();
        // properties
        this.viewHeight = window.innerHeight - 70 - 48 - 48;
        this.viewWidth = window.innerWidth;
        this.pixelRatio = window.devicePixelRatio;
        this.mouse = new three__WEBPACK_IMPORTED_MODULE_0__.Vector2();
        this.touch = new three__WEBPACK_IMPORTED_MODULE_0__.Vector2();
        // skeleton
        this.dots = [];
        this.dotsMapping = [];
        this.lines = [];
        this.exName = '';
        this.playbackSpeed = 1;
        this.isPlaybackSpeedShowing = false;
        this.isCropping = false;
        this.cropStart = -1;
        this.cropEnd = -1;
        this.playPosition = 1;
        this.isPlaying = false;
        this.max = 100;
        this.selectedJoint = '';
        this.angleIndex = 0;
        this.stages = [];
        this.stageIndex = -1;
        this.isSaving = false;
        this.angles = [];
        this.color = ['#f72585', '#4361ee', '#4cc9f0', '#b5179e', '#4895ef', '#7209b7'];
        this.anglesMerged = [];
        this.displayedColumns = ['start', 'mid', 'end'];
        const width = window.innerWidth;
        const height = window.innerHeight;
        this.renderer = new three__WEBPACK_IMPORTED_MODULE_0__.WebGLRenderer({ antialias: true });
        this.scene = new three__WEBPACK_IMPORTED_MODULE_0__.Scene();
        this.camera = new three__WEBPACK_IMPORTED_MODULE_0__.PerspectiveCamera(25, (width * 2 / 3) / height, 0.1, 1000);
        this.orbitControls = new three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_1__.OrbitControls(this.camera, this.renderer.domElement);
        this.orbitControls.update();
        this.gui = new dat_gui__WEBPACK_IMPORTED_MODULE_2__.GUI({ autoPlace: false });
    }
    // event listeners
    onWindowResize(event) {
        this.viewHeight = window.innerHeight - 70 - 48 - 48;
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
        // this.editorContainer.nativeElement.addEventListener('click', (event: MouseEvent) => { this.addJointToStage(this.selectedJoint); });
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
        this.playbackSpeed = this.coordinatesService.playbackSpeed;
        document.getElementById('menuContainer')?.appendChild(this.gui.domElement);
        this.initMenu();
        this.initScene();
    }
    initScene() {
        this.renderer.setClearColor(0xffffff);
        this.camera.position.set(20, 20, 20);
        this.orbitControls.enablePan = true;
        this.orbitControls.enableZoom = true;
        this.orbitControls.enableKeys = true;
        const axesHelper = new three__WEBPACK_IMPORTED_MODULE_0__.AxesHelper(5);
        this.scene.add(axesHelper);
        const ambientLight = new three__WEBPACK_IMPORTED_MODULE_0__.AmbientLight(0xffffff);
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
            const geometry = new three__WEBPACK_IMPORTED_MODULE_0__.SphereBufferGeometry(0.1, 32, 32);
            const material = new three__WEBPACK_IMPORTED_MODULE_0__.MeshBasicMaterial({ color: 0xafaab9 });
            const dot = new three__WEBPACK_IMPORTED_MODULE_0__.Mesh(geometry, material);
            dot.name = point.toString();
            this.dots.push({ index: point, dot });
            this.scene.add(dot);
        }
        // tslint:disable-next-line: forin
        for (const start in this.coordinatesService.connections) {
            // tslint:disable-next-line: forin
            for (const end of this.coordinatesService.connections[start]) {
                const material = new three__WEBPACK_IMPORTED_MODULE_0__.LineBasicMaterial({
                    color: 0x3f51b5,
                    linewidth: 4.0
                });
                const geometry = new three__WEBPACK_IMPORTED_MODULE_0__.BufferGeometry();
                const line = new three__WEBPACK_IMPORTED_MODULE_0__.Line(geometry, material);
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
            const start = new three__WEBPACK_IMPORTED_MODULE_0__.Vector3(x0, y0, z0);
            const end = new three__WEBPACK_IMPORTED_MODULE_0__.Vector3(x1, y1, z1);
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
        this.playbackSpeed = $event;
    }
    setPlayPosition($event) {
        this.coordinatesService.setPlayPosition(Math.round($event.value));
    }
    selectJoint() {
        this.raycaster.setFromCamera(this.mouse, this.camera);
        const mouseIntersection = this.raycaster.intersectObjects(this.scene.children);
        let go = '';
        for (const intersected of mouseIntersection) {
            if (intersected.object instanceof three__WEBPACK_IMPORTED_MODULE_0__.Mesh) {
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
    addToStages() {
        const pose = this.coordinatesService.recording[this.coordinatesService.ri][1];
        this.stages.push([pose, new Set()]);
        this.stageIndex = this.stages.length - 1;
        this.anglesMerged.push([]);
    }
    addJointToStage(joint) {
        if (joint.length > 0 && this.stageIndex >= 0) {
            this.stages[this.stageIndex][1].add(joint);
            const last = this.angles.length - 1;
            switch (this.angleIndex) {
                case 0:
                    this.angles.push([this.stageIndex, [joint]]);
                    this.angleIndex++;
                    break;
                case 1:
                    this.angles[last][1].push(joint);
                    this.angleIndex++;
                    break;
                case 2:
                    this.angles[last][1].push(joint);
                    this.angleIndex = 0;
                    break;
            }
            this.stages.forEach((el, index) => {
                const stageAngles = this.angles.filter(el => el[0] === index).map(el => el[1]);
                this.anglesMerged[index] = stageAngles;
            });
        }
        console.log(this.anglesMerged);
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
    setStageIndex(index) {
        this.stageIndex = index;
    }
    showSave() {
        if (this.isSaving) {
            this.isSaving = false;
            this.isCropping = false;
            this.isPlaybackSpeedShowing = false;
        }
        else {
            this.isSaving = true;
            this.isCropping = false;
            this.isPlaybackSpeedShowing = false;
        }
    }
    onSave() {
        const interimStagesDict = [];
        this.stages.forEach((el, index) => {
            const anglesMerged = this.angles.filter(el => {
                return el[0] === index;
            }).map(el => el[1]);
            interimStagesDict.push({ skeleton: el[0], angles: anglesMerged });
        });
        console.log(interimStagesDict);
        if (this.exName === '') {
            this._snackbar.open('Exercise name not set', 'OK', { duration: 2000 });
        }
        else {
            const toSave = [];
            this.stages.forEach((stage) => {
                toSave.push([stage[0], Array.from(stage[1])]);
            });
            this.coordinatesService.saveRecordingWithStages(interimStagesDict, this.exName);
        }
    }
    crop() {
        if (this.isCropping) {
            this.isCropping = false;
            this.isPlaybackSpeedShowing = false;
        }
        else if (!this.isCropping) {
            this.isPlaybackSpeedShowing = false;
            this.isCropping = true;
            this.coordinatesService.stopPlayingRecording();
            this.isPlaying = false;
        }
    }
    showSpeedSlider() {
        if (this.isPlaybackSpeedShowing) {
            this.isCropping = false;
            this.isPlaybackSpeedShowing = false;
        }
        else if (!this.isPlaybackSpeedShowing) {
            this.isPlaybackSpeedShowing = true;
            this.isCropping = false;
        }
    }
    setCropStart() {
        if (this.cropEnd === -1) {
            this.cropStart = this.coordinatesService.ri;
        }
        else if (this.cropEnd <= this.coordinatesService.ri) {
            this._snackbar.open('Start was setted to be after recording end', 'OK', { duration: 2000 });
        }
        else {
            this.cropStart = this.coordinatesService.ri;
        }
    }
    setCropEnd() {
        if (this.cropStart === -1) {
            this.cropEnd = this.coordinatesService.ri;
        }
        else if (this.cropStart >= this.coordinatesService.ri) {
            this._snackbar.open('End was setted to be after recording start', 'OK', { duration: 2000 });
        }
        else {
            this.cropEnd = this.coordinatesService.ri;
        }
    }
    saveCropping() {
        const croppedRecording = this.coordinatesService.recording.slice(this.cropStart, this.cropEnd + 1);
        this.coordinatesService.ri = 1;
        this.coordinatesService.recording = croppedRecording;
        this.coordinatesService.playRecording();
        this.isPlaying = true;
    }
}
EditorComponent.ɵfac = function EditorComponent_Factory(t) { return new (t || EditorComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵdirectiveInject"](_services_data_service__WEBPACK_IMPORTED_MODULE_3__.DataService), _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵdirectiveInject"](_services_coordinates_service__WEBPACK_IMPORTED_MODULE_4__.CoordinatesService), _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵdirectiveInject"](_angular_material_snack_bar__WEBPACK_IMPORTED_MODULE_6__.MatSnackBar)); };
EditorComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵdefineComponent"]({ type: EditorComponent, selectors: [["app-editor"]], viewQuery: function EditorComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵviewQuery"](_c0, 5);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵloadQuery"]()) && (ctx.editorContainer = _t.first);
    } }, hostBindings: function EditorComponent_HostBindings(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("resize", function EditorComponent_resize_HostBindingHandler($event) { return ctx.onWindowResize($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵresolveWindow"])("mousemove", function EditorComponent_mousemove_HostBindingHandler($event) { return ctx.onDocumentMouseMove($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵresolveDocument"])("touchend", function EditorComponent_touchend_HostBindingHandler($event) { return ctx.onTouchEnd($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵresolveDocument"]);
    } }, decls: 73, vars: 30, consts: [["cols", "3", 3, "rowHeight"], ["id", "rc", 3, "colspan", "rowspan"], ["id", "editorContainer", 1, "div-editor"], ["editorContainer", ""], [3, "colspan", "rowspan"], ["mat-stretch-tabs", "", 1, "mat-elevation-z4"], ["label", "Winkel"], ["mat-fab", "", "color", "primary", "aria-label", "Example icon button with a delete icon", 2, "margin", "10px", 3, "click"], [4, "ngFor", "ngForOf"], ["label", "Abstand"], ["label", "H\u00F6he"], ["style", "width: 100%; display: flex; align-items: flex-start !important;", 4, "ngIf"], [1, "player", 2, "padding-left", "8px"], ["mat-mini-fab", "", "color", "primary", 3, "click"], [4, "ngIf"], [3, "ngModel", "min", "max", "ngClass", "ngModelChange", "input"], ["mat-icon-button", "", "aria-label", "Example icon-button with a menu", 3, "matMenuTriggerFor"], ["menu", "matMenu"], ["mat-menu-item", "", 3, "matMenuTriggerFor"], ["mat-menu-item", ""], [3, "ngModel", "max", "min", "step", "ngModelChange"], ["mat-raised-button", "", "color", "primary", 1, "action-buttons", 3, "click"], ["style", "margin-left: 10px;", 4, "ngIf"], ["cropMenu", "matMenu"], ["mat-menu-item", "", 3, "click"], [2, "width", "100%", "display", "flex", "align-items", "flex-start !important"], [1, "mat-elevation-z0"], [2, "margin-left", "10px"], ["appearance", "standard"], ["matInput", "", "placeholder", "Bsp. Squats", 3, "ngModel", "ngModelChange"], ["mat-icon-button", "", "matSuffix", "", 3, "click"], ["matSuffix", ""]], template: function EditorComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](0, "mat-grid-list", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](1, "mat-grid-tile", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelement"](2, "div", 2, 3);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](4, "mat-grid-tile", 4);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](5, "mat-tab-group", 5);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](6, "mat-tab", 6);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](7, "div");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](8, "button", 7);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_8_listener() { return ctx.addJointToStage(ctx.selectedJoint); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](9, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](10, "add");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](11, "span");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](12);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](13, EditorComponent_p_13_Template, 3, 2, "p", 8);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](14, "mat-tab", 9);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](15, "div");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](16, "button", 7);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_16_listener() { return ctx.addJointToStage(ctx.selectedJoint); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](17, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](18, "add");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](19, "span");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](20);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](21, EditorComponent_p_21_Template, 3, 2, "p", 8);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](22, "mat-tab", 10);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](23, "div");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](24, "button", 7);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_24_listener() { return ctx.addJointToStage(ctx.selectedJoint); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](25, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](26, "add");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](27, "span");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](28);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelement"](29, "mat-grid-tile");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](30, "mat-grid-tile", 4);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](31, EditorComponent_div_31_Template, 8, 1, "div", 11);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](32, "div", 12);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](33, "button", 13);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_33_listener() { return ctx.playPause(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](34, EditorComponent_mat_icon_34_Template, 2, 0, "mat-icon", 14);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](35, EditorComponent_mat_icon_35_Template, 2, 0, "mat-icon", 14);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](36, "mat-slider", 15);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("ngModelChange", function EditorComponent_Template_mat_slider_ngModelChange_36_listener($event) { return ctx.playPosition = $event; })("input", function EditorComponent_Template_mat_slider_input_36_listener($event) { return ctx.setPlayPosition($event); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](37, "button", 16);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](38, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](39, "more_vert");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](40, "mat-menu", null, 17);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](42, "button", 18);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](43, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](44, "content_cut");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](45, "span");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](46, "Crop");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](47, "button", 19);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](48, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](49, "speed");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](50, "span");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](51, "mat-slider", 20);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("ngModelChange", function EditorComponent_Template_mat_slider_ngModelChange_51_listener($event) { return ctx.setSpeed($event); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](52);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](53, "button", 19);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](54, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](55, "file_download");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](56, "span");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](57, "Export JSON ");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](58, "button", 21);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_58_listener() { return ctx.showSave(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](59, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](60, "save");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](61, "Save ");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtemplate"](62, EditorComponent_span_62_Template, 8, 1, "span", 22);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](63, "mat-menu", null, 23);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](65, "button", 24);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_65_listener() { return ctx.setCropStart(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](66, " Set Start ");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](67, "button", 24);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_67_listener() { return ctx.setCropStart(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](68, " Set End ");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](69, "button", 24);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵlistener"]("click", function EditorComponent_Template_button_click_69_listener() { return ctx.saveCropping(); });
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementStart"](70, "mat-icon");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](71, "save");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtext"](72, " Save ");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵelementEnd"]();
    } if (rf & 2) {
        const _r6 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵreference"](41);
        const _r8 = _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵreference"](64);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("rowHeight", ctx.viewHeight / 8);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("colspan", 2)("rowspan", 8);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](3);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("colspan", 1)("rowspan", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](8);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate"](ctx.selectedJoint);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngForOf", ctx.anglesMerged[0]);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](7);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate"](ctx.selectedJoint);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngForOf", ctx.anglesMerged[0]);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](7);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate"](ctx.selectedJoint);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](2);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("colspan", 1)("rowspan", 5);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngIf", ctx.anglesMerged.length > 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](3);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngIf", !ctx.isPlaying);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngIf", ctx.isPlaying);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngModel", ctx.playPosition)("min", 1)("max", ctx.max)("ngClass", _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵpureFunction2"](27, _c1, !ctx.isSaving, ctx.isSaving));
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("matMenuTriggerFor", _r6);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](5);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("matMenuTriggerFor", _r8);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](9);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngModel", ctx.playbackSpeed)("max", 2)("min", 0.1)("step", 0.1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](1);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵtextInterpolate1"]("", ctx.playbackSpeed, "x ");
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵadvance"](10);
        _angular_core__WEBPACK_IMPORTED_MODULE_5__["ɵɵproperty"]("ngIf", ctx.isSaving);
    } }, directives: [_angular_material_grid_list__WEBPACK_IMPORTED_MODULE_7__.MatGridList, _angular_material_grid_list__WEBPACK_IMPORTED_MODULE_7__.MatGridTile, _angular_material_tabs__WEBPACK_IMPORTED_MODULE_8__.MatTabGroup, _angular_material_tabs__WEBPACK_IMPORTED_MODULE_8__.MatTab, _angular_material_button__WEBPACK_IMPORTED_MODULE_9__.MatButton, _angular_material_icon__WEBPACK_IMPORTED_MODULE_10__.MatIcon, _angular_common__WEBPACK_IMPORTED_MODULE_11__.NgForOf, _angular_common__WEBPACK_IMPORTED_MODULE_11__.NgIf, _angular_material_slider__WEBPACK_IMPORTED_MODULE_12__.MatSlider, _angular_forms__WEBPACK_IMPORTED_MODULE_13__.NgControlStatus, _angular_forms__WEBPACK_IMPORTED_MODULE_13__.NgModel, _angular_common__WEBPACK_IMPORTED_MODULE_11__.NgClass, _angular_material_menu__WEBPACK_IMPORTED_MODULE_14__.MatMenuTrigger, _angular_material_menu__WEBPACK_IMPORTED_MODULE_14__.MatMenu, _angular_material_menu__WEBPACK_IMPORTED_MODULE_14__.MatMenuItem, _angular_material_card__WEBPACK_IMPORTED_MODULE_15__.MatCard, _angular_material_card__WEBPACK_IMPORTED_MODULE_15__.MatCardHeader, _angular_material_card__WEBPACK_IMPORTED_MODULE_15__.MatCardTitle, _angular_material_card__WEBPACK_IMPORTED_MODULE_15__.MatCardContent, _angular_material_card__WEBPACK_IMPORTED_MODULE_15__.MatCardActions, _angular_material_form_field__WEBPACK_IMPORTED_MODULE_16__.MatFormField, _angular_material_form_field__WEBPACK_IMPORTED_MODULE_16__.MatLabel, _angular_material_input__WEBPACK_IMPORTED_MODULE_17__.MatInput, _angular_forms__WEBPACK_IMPORTED_MODULE_13__.DefaultValueAccessor, _angular_material_form_field__WEBPACK_IMPORTED_MODULE_16__.MatSuffix], styles: ["@charset \"UTF-8\";\n.div-editor[_ngcontent-%COMP%] {\n  position: relative;\n  display: flex;\n  justify-content: center;\n}\n[_nghost-%COMP%]    >  >  > .mat-figure[_ngcontent-%COMP%] {\n  align-items: start !important;\n}\n.mat-icon[_ngcontent-%COMP%] {\n  vertical-align: middle;\n}\n.speed-span[_ngcontent-%COMP%] {\n  margin-left: 0px;\n}\n.mat-card[_ngcontent-%COMP%] {\n  width: 100%;\n}\n.stage-joints[_ngcontent-%COMP%] {\n  width: 100%;\n}\n.player[_ngcontent-%COMP%]\u00A0 {\n  margin: 10;\n  display: inline-block;\n}\n.play-slider-big[_ngcontent-%COMP%] {\n  margin-left: 8px;\n  width: calc(100% - 48px - 40px - 10px - 100px);\n}\n.play-slider-small[_ngcontent-%COMP%] {\n  margin-left: 8px;\n  width: calc(100% - 48px - 40px - 10px - 320px);\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImVkaXRvci5jb21wb25lbnQuc2NzcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQSxnQkFBZ0I7QUFBaEI7RUFDRSxrQkFBQTtFQUNBLGFBQUE7RUFDQSx1QkFBQTtBQUVGO0FBQ0E7RUFDRSw2QkFBQTtBQUVGO0FBQ0E7RUFDRSxzQkFBQTtBQUVGO0FBQ0E7RUFDRSxnQkFBQTtBQUVGO0FBQ0E7RUFDRSxXQUFBO0FBRUY7QUFBQTtFQUNFLFdBQUE7QUFHRjtBQUFBO0VBQ0UsVUFBQTtFQUNBLHFCQUFBO0FBR0Y7QUFBQTtFQUNFLGdCQUFBO0VBQ0EsOENBQUE7QUFHRjtBQUFBO0VBQ0UsZ0JBQUE7RUFDQSw4Q0FBQTtBQUdGIiwiZmlsZSI6ImVkaXRvci5jb21wb25lbnQuc2NzcyIsInNvdXJjZXNDb250ZW50IjpbIi5kaXYtZWRpdG9yIHtcbiAgcG9zaXRpb246IHJlbGF0aXZlO1xuICBkaXNwbGF5OiBmbGV4O1xuICBqdXN0aWZ5LWNvbnRlbnQ6IGNlbnRlcjtcbn1cblxuOmhvc3QgPj4+IC5tYXQtZmlndXJle1xuICBhbGlnbi1pdGVtczogc3RhcnQgIWltcG9ydGFudDtcbn1cblxuLm1hdC1pY29uIHtcbiAgdmVydGljYWwtYWxpZ246IG1pZGRsZTtcbn1cblxuLnNwZWVkLXNwYW4ge1xuICBtYXJnaW4tbGVmdDogMHB4O1xufVxuXG4ubWF0LWNhcmQge1xuICB3aWR0aDogMTAwJTtcbn1cbi5zdGFnZS1qb2ludHMge1xuICB3aWR0aDogMTAwJTtcbn1cblxuLnBsYXllcsKge1xuICBtYXJnaW46IDEwO1xuICBkaXNwbGF5OmlubGluZS1ibG9jaztcbn1cblxuLnBsYXktc2xpZGVyLWJpZyB7XG4gIG1hcmdpbi1sZWZ0OiA4cHg7XG4gIHdpZHRoOiBjYWxjKDEwMCUgLSA0OHB4IC0gNDBweCAtIDEwcHggLSAxMDBweCk7XG59XG5cbi5wbGF5LXNsaWRlci1zbWFsbCB7XG4gIG1hcmdpbi1sZWZ0OiA4cHg7XG4gIHdpZHRoOiBjYWxjKDEwMCUgLSA0OHB4IC0gNDBweCAtIDEwcHggLSAzMjBweCk7XG59XG4iXX0= */"] });


/***/ }),

/***/ 8802:
/*!**************************************!*\
  !*** ./src/app/edk/edk.component.ts ***!
  \**************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "EDKComponent": () => (/* binding */ EDKComponent)
/* harmony export */ });
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/core */ 7716);
/* harmony import */ var _angular_material_tabs__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/material/tabs */ 5939);
/* harmony import */ var _editor_editor_component__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./editor/editor.component */ 142);
/* harmony import */ var _labeler_labeler_component__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./labeler/labeler.component */ 881);




class EDKComponent {
    constructor() { }
    ngOnInit() {
    }
}
EDKComponent.ɵfac = function EDKComponent_Factory(t) { return new (t || EDKComponent)(); };
EDKComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵdefineComponent"]({ type: EDKComponent, selectors: [["app-edk"]], decls: 7, vars: 0, consts: [["mat-stretch-tabs", "", 1, "example-stretched-tabs", "mat-elevation-z4"], ["label", "Editor"], ["label", "Labeler"], ["label", "Browser"]], template: function EDKComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelementStart"](0, "mat-tab-group", 0);
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelementStart"](1, "mat-tab", 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelement"](2, "app-editor");
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelementStart"](3, "mat-tab", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelement"](4, "app-labeler");
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelementStart"](5, "mat-tab", 3);
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵtext"](6, " Content 3 ");
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵelementEnd"]();
    } }, directives: [_angular_material_tabs__WEBPACK_IMPORTED_MODULE_3__.MatTabGroup, _angular_material_tabs__WEBPACK_IMPORTED_MODULE_3__.MatTab, _editor_editor_component__WEBPACK_IMPORTED_MODULE_0__.EditorComponent, _labeler_labeler_component__WEBPACK_IMPORTED_MODULE_1__.LabelerComponent], styles: ["\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IiIsImZpbGUiOiJlZGsuY29tcG9uZW50LnNjc3MifQ== */"] });


/***/ }),

/***/ 881:
/*!**************************************************!*\
  !*** ./src/app/edk/labeler/labeler.component.ts ***!
  \**************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "LabelerComponent": () => (/* binding */ LabelerComponent)
/* harmony export */ });
/* harmony import */ var three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! three/examples/jsm/controls/OrbitControls.js */ 6887);
/* harmony import */ var three_examples_jsm_controls_DragControls_js__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! three/examples/jsm/controls/DragControls.js */ 496);
/* harmony import */ var three__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! three */ 7758);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/core */ 7716);




const _c0 = ["labelerContainer"];
const _c1 = ["video"];
class LabelerComponent {
    constructor() {
        this.mouse = new three__WEBPACK_IMPORTED_MODULE_2__.Vector2();
        this.raycaster = new three__WEBPACK_IMPORTED_MODULE_2__.Raycaster();
        console.log(this.videoContainer);
        const width = window.innerWidth;
        const height = window.innerHeight - 64;
        this.renderer = new three__WEBPACK_IMPORTED_MODULE_2__.WebGLRenderer({ antialias: true });
        this.scene = new three__WEBPACK_IMPORTED_MODULE_2__.Scene();
        this.camera = new three__WEBPACK_IMPORTED_MODULE_2__.PerspectiveCamera(25, width / height, 0.1, 1000);
        this.orbitControls = new three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_0__.OrbitControls(this.camera, this.renderer.domElement);
        const geometry = new three__WEBPACK_IMPORTED_MODULE_2__.SphereBufferGeometry(0.1, 32, 32);
        const material = new three__WEBPACK_IMPORTED_MODULE_2__.MeshBasicMaterial({ color: 0xafaab9 });
        const dot = new three__WEBPACK_IMPORTED_MODULE_2__.Mesh(geometry, material);
        dot.position.x = 0;
        dot.position.z = 3;
        dot.position.y = 0;
        this.scene.add(dot);
        this.dragControls = new three_examples_jsm_controls_DragControls_js__WEBPACK_IMPORTED_MODULE_1__.DragControls([dot], this.camera, this.renderer.domElement);
        this.dragControls.addEventListener('drag', () => this.renderer.render(this.scene, this.camera));
        this.orbitControls.update();
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
    onDocumentMouseClick(event) {
        event.preventDefault();
        const draggableObjects = this.dragControls.getObjects();
        draggableObjects.length = 0;
        this.mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
        this.mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
        this.raycaster.setFromCamera(this.mouse, this.camera);
        const intersections = this.raycaster.intersectObjects(this.scene.children, true);
        if (intersections.length > 0) {
        }
    }
    handleKeyboardEvent(event) {
        if (event.key === 'c' && event.ctrlKey) {
            this.switchControls();
        }
    }
    ngAfterViewInit() {
        this.camera.lookAt(this.scene.position);
        this.rendererContainer.nativeElement.appendChild(this.renderer.domElement);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.setSize(window.innerWidth, (window.innerHeight - 64));
        this.initVideo();
        window.requestAnimationFrame(() => this.animate());
    }
    animate() {
        this.selectObject();
        window.requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }
    ngOnInit() {
        this.initScene();
    }
    initVideo() {
        this.videoContainer.nativeElement.play();
        this.videoContainer.nativeElement.addEventListener('play', () => {
            console.log('playing');
        });
        const texture = new three__WEBPACK_IMPORTED_MODULE_2__.VideoTexture(this.videoContainer.nativeElement);
        texture.needsUpdate = true;
        const geometry = new three__WEBPACK_IMPORTED_MODULE_2__.BoxGeometry(16, 9, 0);
        const material = new three__WEBPACK_IMPORTED_MODULE_2__.MeshLambertMaterial({ color: 0xffffff, map: texture });
        const mesh = new three__WEBPACK_IMPORTED_MODULE_2__.Mesh(geometry, material);
        mesh.position.x = 0;
        mesh.position.y = 0;
        mesh.position.z = 0;
        this.scene.add(mesh);
    }
    initScene() {
        this.renderer.setClearColor(0xffffff);
        this.camera.position.set(20, 20, 20);
        this.orbitControls.enablePan = true;
        this.orbitControls.enableZoom = true;
        this.orbitControls.enableKeys = true;
        this.orbitControls.enabled = false;
        this.dragControls.enabled = true;
        const axesHelper = new three__WEBPACK_IMPORTED_MODULE_2__.AxesHelper(5);
        this.scene.add(axesHelper);
        const ambientLight = new three__WEBPACK_IMPORTED_MODULE_2__.AmbientLight(0xffffff);
        ambientLight.intensity = 2;
        this.scene.add(ambientLight);
    }
    selectObject() {
        this.raycaster.setFromCamera(this.mouse, this.camera);
        const mouseIntersection = this.raycaster.intersectObjects(this.scene.children);
        let go = '';
        for (const intersected of mouseIntersection) {
            if (intersected.object instanceof three__WEBPACK_IMPORTED_MODULE_2__.Mesh) {
                go = intersected.object.name;
            }
        }
    }
    switchControls() {
        if (this.orbitControls.enabled) {
            this.orbitControls.enabled = false;
            this.dragControls.enabled = true;
        }
        else {
            this.orbitControls.enabled = true;
            this.dragControls.enabled = false;
        }
    }
}
LabelerComponent.ɵfac = function LabelerComponent_Factory(t) { return new (t || LabelerComponent)(); };
LabelerComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵdefineComponent"]({ type: LabelerComponent, selectors: [["app-labeler"]], viewQuery: function LabelerComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵviewQuery"](_c0, 5);
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵviewQuery"](_c1, 5);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵloadQuery"]()) && (ctx.rendererContainer = _t.first);
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵloadQuery"]()) && (ctx.videoContainer = _t.first);
    } }, hostBindings: function LabelerComponent_HostBindings(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵlistener"]("resize", function LabelerComponent_resize_HostBindingHandler($event) { return ctx.onWindowResize($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵresolveWindow"])("mousemove", function LabelerComponent_mousemove_HostBindingHandler($event) { return ctx.onDocumentMouseMove($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵresolveDocument"])("mouseclick", function LabelerComponent_mouseclick_HostBindingHandler($event) { return ctx.onDocumentMouseClick($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵresolveDocument"])("keypress", function LabelerComponent_keypress_HostBindingHandler($event) { return ctx.handleKeyboardEvent($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵresolveDocument"]);
    } }, decls: 6, vars: 0, consts: [["id", "labelerContainer", 1, "div-labeler"], ["labelerContainer", ""], ["id", "guiContainer"], ["id", "video", "loop", "", "muted", "", "playsinline", "", 2, "display", "none"], ["video", ""], ["src", "../../assets/files/gym.mp4", "type", "video/mp4"]], template: function LabelerComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵelementStart"](0, "div", 0, 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵelement"](2, "div", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵelementEnd"]();
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵelementStart"](3, "video", 3, 4);
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵelement"](5, "source", 5);
        _angular_core__WEBPACK_IMPORTED_MODULE_3__["ɵɵelementEnd"]();
    } }, styles: ["\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IiIsImZpbGUiOiJsYWJlbGVyLmNvbXBvbmVudC5zY3NzIn0= */"] });


/***/ }),

/***/ 6632:
/*!****************************************************!*\
  !*** ./src/app/edk/renderer/renderer.component.ts ***!
  \****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "RendererComponent": () => (/* binding */ RendererComponent)
/* harmony export */ });
/* harmony import */ var three__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! three */ 7758);
/* harmony import */ var three_examples_jsm_loaders_GLTFLoader_js__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! three/examples/jsm/loaders/GLTFLoader.js */ 5192);
/* harmony import */ var three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! three/examples/jsm/controls/OrbitControls.js */ 6887);
/* harmony import */ var three_examples_jsm_controls_TransformControls_js__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! three/examples/jsm/controls/TransformControls.js */ 178);
/* harmony import */ var dat_gui__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! dat.gui */ 4486);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! @angular/core */ 7716);
/* harmony import */ var _services_data_service__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ../../services/data.service */ 2468);
/* harmony import */ var _services_coordinates_service__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! ../../services/coordinates.service */ 8498);








const _c0 = ["rendererContainer"];
class RendererComponent {
    constructor(dataService, coordinatesService) {
        this.dataService = dataService;
        this.coordinatesService = coordinatesService;
        // properties
        this.pixelRatio = window.devicePixelRatio;
        this.mouse = new three__WEBPACK_IMPORTED_MODULE_0__.Vector2();
        this.touch = new three__WEBPACK_IMPORTED_MODULE_0__.Vector2();
        // gltf loader
        this.loader = new three_examples_jsm_loaders_GLTFLoader_js__WEBPACK_IMPORTED_MODULE_1__.GLTFLoader();
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
        this.renderer = new three__WEBPACK_IMPORTED_MODULE_0__.WebGLRenderer({ antialias: true });
        this.scene = new three__WEBPACK_IMPORTED_MODULE_0__.Scene();
        this.camera = new three__WEBPACK_IMPORTED_MODULE_0__.PerspectiveCamera(25, width / height, 0.1, 1000);
        this.clock = new three__WEBPACK_IMPORTED_MODULE_0__.Clock();
        this.model = new three__WEBPACK_IMPORTED_MODULE_0__.Object3D();
        this.mixer = new three__WEBPACK_IMPORTED_MODULE_0__.AnimationMixer(this.model);
        this.orbitControls = new three_examples_jsm_controls_OrbitControls_js__WEBPACK_IMPORTED_MODULE_2__.OrbitControls(this.camera, this.renderer.domElement);
        this.orbitControls.update();
        this.gui = new dat_gui__WEBPACK_IMPORTED_MODULE_4__.GUI({ autoPlace: false });
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
        document.getElementById('guiContainer')?.appendChild(this.gui.domElement);
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
        const axesHelper = new three__WEBPACK_IMPORTED_MODULE_0__.AxesHelper(5);
        this.scene.add(axesHelper);
        const ambientLight = new three__WEBPACK_IMPORTED_MODULE_0__.AmbientLight(0xffffff);
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
                const mesh = new three__WEBPACK_IMPORTED_MODULE_0__.Mesh();
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
        // const skeletoncheck = this.gui.add(menu, 'skeleton', false).name('Show Skeleton');
        // skeletoncheck.onChange(val => { this.skeletonCheck(val); });
        const startRecording = this.gui.add(menu, 'startRecording').name('🎥');
        const stopRecording = this.gui.add(menu, 'stopRecording').name('🎬');
        const saveRecording = this.gui.add(menu, 'saveRecording').name('💾');
        // const deleteBones = this.gui.add(menu, 'deleteBones').name('Delete Not Used Bones');
        const resetButton = this.gui.add(menu, 'reset').name('Reset');
        // const editButton = this.gui.add(menu, 'edit').name('Edit');
        const renderSceleton = this.gui.add(menu, 'renderSkeleton').name('Render Skeleton');
    }
    deleteBones() {
        this.model.traverse(obj => {
            if (obj instanceof three__WEBPACK_IMPORTED_MODULE_0__.Bone) {
                if (obj.name.startsWith('f_') || obj.name.startsWith('thumb') || obj.name.startsWith('palm')) {
                    console.log('deleted');
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
            this.mixer = new three__WEBPACK_IMPORTED_MODULE_0__.AnimationMixer(gltf.scene);
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
            const skeletonHelper = new three__WEBPACK_IMPORTED_MODULE_0__.SkeletonHelper(this.model);
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
            if (obj instanceof three__WEBPACK_IMPORTED_MODULE_0__.Bone) {
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
        const transformControl = new three_examples_jsm_controls_TransformControls_js__WEBPACK_IMPORTED_MODULE_3__.TransformControls(this.camera, this.renderer.domElement);
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
        for (const line of this.lines) {
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
            const geometry = new three__WEBPACK_IMPORTED_MODULE_0__.SphereBufferGeometry(0.1, 32, 32);
            const material = new three__WEBPACK_IMPORTED_MODULE_0__.MeshBasicMaterial({ color: 0xafaab9 });
            const dot = new three__WEBPACK_IMPORTED_MODULE_0__.Mesh(geometry, material);
            dot.name = point.toString();
            this.dots.push({ index: point, dot });
            this.scene.add(dot);
        }
        // tslint:disable-next-line: forin
        for (const start in this.coordinatesService.connections) {
            // tslint:disable-next-line: forin
            for (const end of this.coordinatesService.connections[start]) {
                const material = new three__WEBPACK_IMPORTED_MODULE_0__.LineBasicMaterial({
                    color: 0x3f51b5,
                    linewidth: 4.0
                });
                const geometry = new three__WEBPACK_IMPORTED_MODULE_0__.BufferGeometry();
                const line = new three__WEBPACK_IMPORTED_MODULE_0__.Line(geometry, material);
                this.lines.push(line);
                this.dotsMapping.push([start, end]);
                this.scene.add(line);
            }
        }
        this.renderSkeleton(5, 0, 0, 0);
        this.coordinatesService.update.subscribe(() => {
            // this.transferCoordinates();
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
            const start = new three__WEBPACK_IMPORTED_MODULE_0__.Vector3(x0, y0, z0);
            const end = new three__WEBPACK_IMPORTED_MODULE_0__.Vector3(x1, y1, z1);
            line.geometry.setFromPoints([start, end]);
            line.geometry.computeBoundingBox();
            line.geometry.computeBoundingSphere();
        }
    }
}
RendererComponent.ɵfac = function RendererComponent_Factory(t) { return new (t || RendererComponent)(_angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵdirectiveInject"](_services_data_service__WEBPACK_IMPORTED_MODULE_5__.DataService), _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵdirectiveInject"](_services_coordinates_service__WEBPACK_IMPORTED_MODULE_6__.CoordinatesService)); };
RendererComponent.ɵcmp = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵdefineComponent"]({ type: RendererComponent, selectors: [["app-renderer"]], viewQuery: function RendererComponent_Query(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵviewQuery"](_c0, 5);
    } if (rf & 2) {
        let _t;
        _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵqueryRefresh"](_t = _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵloadQuery"]()) && (ctx.rendererContainer = _t.first);
    } }, hostBindings: function RendererComponent_HostBindings(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵlistener"]("resize", function RendererComponent_resize_HostBindingHandler($event) { return ctx.onWindowResize($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵresolveWindow"])("mousemove", function RendererComponent_mousemove_HostBindingHandler($event) { return ctx.onDocumentMouseMove($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵresolveDocument"])("touchend", function RendererComponent_touchend_HostBindingHandler($event) { return ctx.onTouchEnd($event); }, false, _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵresolveDocument"]);
    } }, decls: 3, vars: 0, consts: [["id", "rendererContainer", 1, "div-renderer"], ["rendererContainer", ""], ["id", "guiContainer"]], template: function RendererComponent_Template(rf, ctx) { if (rf & 1) {
        _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵelementStart"](0, "div", 0, 1);
        _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵelement"](2, "div", 2);
        _angular_core__WEBPACK_IMPORTED_MODULE_7__["ɵɵelementEnd"]();
    } }, styles: [".div-renderer[_ngcontent-%COMP%] {\n  position: relative;\n  display: flex;\n  justify-content: center;\n}\n\n#guiContainer[_ngcontent-%COMP%] {\n  position: absolute;\n  top: 0em;\n  right: 0em;\n  z-index: 1;\n}\n/*# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInJlbmRlcmVyLmNvbXBvbmVudC5zY3NzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBO0VBQ0Usa0JBQUE7RUFDQSxhQUFBO0VBQ0EsdUJBQUE7QUFDRjs7QUFFQTtFQUNFLGtCQUFBO0VBQ0EsUUFBQTtFQUNBLFVBQUE7RUFDQSxVQUFBO0FBQ0YiLCJmaWxlIjoicmVuZGVyZXIuY29tcG9uZW50LnNjc3MiLCJzb3VyY2VzQ29udGVudCI6WyIuZGl2LXJlbmRlcmVyIHtcbiAgcG9zaXRpb246IHJlbGF0aXZlO1xuICBkaXNwbGF5OiBmbGV4O1xuICBqdXN0aWZ5LWNvbnRlbnQ6IGNlbnRlcjtcbn1cblxuI2d1aUNvbnRhaW5lciB7XG4gIHBvc2l0aW9uOiBhYnNvbHV0ZTtcbiAgdG9wOiAwZW07XG4gIHJpZ2h0OiAwZW07XG4gIHotaW5kZXg6IDE7XG59XG4iXX0= */"] });


/***/ }),

/***/ 8498:
/*!*************************************************!*\
  !*** ./src/app/services/coordinates.service.ts ***!
  \*************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "CoordinatesService": () => (/* binding */ CoordinatesService)
/* harmony export */ });
/* harmony import */ var rxjs__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! rxjs */ 6215);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/core */ 7716);
/* harmony import */ var _data_service__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./data.service */ 2468);



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
        this.update = new rxjs__WEBPACK_IMPORTED_MODULE_1__.BehaviorSubject(this.lastPose);
        this.play = new rxjs__WEBPACK_IMPORTED_MODULE_1__.BehaviorSubject(this.lastPose);
        this.playPosition = new rxjs__WEBPACK_IMPORTED_MODULE_1__.BehaviorSubject(this.ri);
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
    saveRecordingWithStages(stages, name) {
        this.dataService.saveExerciseWithStages(stages, name, this.recording);
    }
}
CoordinatesService.ɵfac = function CoordinatesService_Factory(t) { return new (t || CoordinatesService)(_angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵinject"](_data_service__WEBPACK_IMPORTED_MODULE_0__.DataService)); };
CoordinatesService.ɵprov = /*@__PURE__*/ _angular_core__WEBPACK_IMPORTED_MODULE_2__["ɵɵdefineInjectable"]({ token: CoordinatesService, factory: CoordinatesService.ɵfac, providedIn: 'root' });


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
/* harmony import */ var rxjs_webSocket__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! rxjs/webSocket */ 2835);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @angular/core */ 7716);
/* harmony import */ var _angular_common_http__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/common/http */ 1841);



class DataService {
    constructor(http) {
        this.http = http;
    }
    getWebsocket() {
        return (0,rxjs_webSocket__WEBPACK_IMPORTED_MODULE_0__.webSocket)('ws://localhost:3000');
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
    saveExerciseWithStages(stages, name, recording) {
        const toSave = {
            'stages': stages,
            'recording': recording,
            'name': name
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
// import 'zone.js/plugins/zone-error';  // Included with Angular CLI.


/***/ }),

/***/ 4431:
/*!*********************!*\
  !*** ./src/main.ts ***!
  \*********************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony import */ var _angular_platform_browser__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @angular/platform-browser */ 9075);
/* harmony import */ var _angular_core__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @angular/core */ 7716);
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
/******/ "use strict";
/******/ 
/******/ var __webpack_exec__ = (moduleId) => (__webpack_require__(__webpack_require__.s = moduleId))
/******/ __webpack_require__.O(0, ["vendor"], () => (__webpack_exec__(4431)));
/******/ var __webpack_exports__ = __webpack_require__.O();
/******/ }
]);
//# sourceMappingURL=main.js.map