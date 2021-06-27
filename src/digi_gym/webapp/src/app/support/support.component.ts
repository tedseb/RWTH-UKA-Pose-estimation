import { Component, OnInit } from '@angular/core';

@Component({
  selector: 'app-support',
  templateUrl: './support.component.html',
  styleUrls: ['./support.component.scss']
})
export class SupportComponent implements OnInit {

  requests =  [{description: "Hey, Station 2 stinkt brutal", user: {name: "Basti"}, date: "Today, 12:14"}, {description: "Moin, kann mir jemand zeigen wie die Übung geht??", user: {name: "Thomas"}, date: "Today, 12:14"}]

  constructor() { }

  ngOnInit(): void {
  }

}
