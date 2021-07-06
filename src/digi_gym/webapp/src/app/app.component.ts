import { Component } from '@angular/core';
import { EmergenciesService } from './services/emergencies.service';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss']
})
export class AppComponent {
  title = 'Digi-Gym';
  emergency = false;

  constructor(private emergencyService: EmergenciesService) {
    this.emergencyService.getAllEmergencies().subscribe(res => {
      console.log(res);
      if(res.filter(x => x.done == false).length >= 1) {
        console.log("emergency");
        this.emergency = true;
      } else {this.emergency = false;}
    });
  }
}
