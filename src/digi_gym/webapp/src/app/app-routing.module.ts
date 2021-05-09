import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { SpotsComponent } from './spots/spots.component';

const routes: Routes = [
  { path: 'spots', component: SpotsComponent },
  { path: '', redirectTo: '/spots', pathMatch: 'full' }
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
