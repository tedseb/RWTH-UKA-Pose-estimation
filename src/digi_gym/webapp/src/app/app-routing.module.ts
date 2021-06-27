import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { AppComponent } from './app.component';
import { LoginComponent } from './auth/login/login.component';
import { SignupComponent } from './auth/signup/signup.component';
import { SocialComponent } from './social/social.component';
import { SingleStationComponent } from './station/single-station/single-station.component';
import { StationComponent } from './station/station.component';
import { StatisticsComponent } from './statistics/statistics.component';
import { SupportComponent } from './support/support.component';

const routes: Routes = [
  { path: 'login', component: LoginComponent },
  { path: 'signup', component: SignupComponent },
  { path: 'stations', component: StationComponent },
  { path: 'statistics', component: StatisticsComponent },
  { path: 'social', component: SocialComponent },
  { path: 'support', component: SupportComponent }
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
