/* Interactive elements
 * Copyright 2016 Harmen de Weerd
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

function addMouseTracker(elem) {
  elem.mouse = {x : 0, y : 0, buttons : 0, targetElement : null, hasFocus: false};
  elem.mouse.interactiveElements = [];
  elem.getInteractiveElement = findElement;
  elem.addEventListener("mousedown",
                        function(event) {
                          this.mouse.buttons = event.buttons;
                          this.mouse.x = event.offsetX;
                          this.mouse.y = event.offsetY;
                          this.mouse.targetElement = this.getInteractiveElement(event.offsetX, event.offsetY);
                          if (this.mouse.targetElement != null) {
                            this.mouse.targetElement.startAction(event);
                          }
                        });
  elem.addEventListener("mouseup",
                        function(event) {
                          this.mouse.buttons = event.buttons;
                          if (this.mouse.targetElement != null) {
                            this.mouse.targetElement.endAction(event);
                            this.mouse.targetElement = null;
                          }
                        });
  elem.addEventListener("mousemove",
                        function(event) {
                          this.mouse.hasFocus = true;
                          this.mouse.x = event.offsetX;
                          this.mouse.y = event.offsetY;
                          if (this.mouse.targetElement != null) {
                            this.mouse.targetElement.continueAction(event);
                          } else {
                            var interactiveElement = this.getInteractiveElement(event.offsetX, event.offsetY);
                            if (interactiveElement != null) {
                              this.style.cursor = interactiveElement.getActionLabel(event.offsetX - interactiveElement.x, event.offsetY - interactiveElement.y);
                            } else {
                              this.style.cursor = "initial";
                            }
                          }
                        });
  elem.addEventListener("mouseout",
                        function(event) {
                          this.mouse.hasFocus = false;
                        });
  elem.addInteractiveElement = function(obj) {
    this.mouse.interactiveElements.push(obj);
  };
}

function findElement(x, y) {
  for (var i = this.mouse.interactiveElements.length - 1; i >= 0; --i) {
    if (x < this.mouse.interactiveElements[i].x + this.mouse.interactiveElements[i].getWidth() && x > this.mouse.interactiveElements[i].x &&
        y < this.mouse.interactiveElements[i].y + this.mouse.interactiveElements[i].getHeight() && y > this.mouse.interactiveElements[i].y) {
      return this.mouse.interactiveElements[i];
    }
  }
  return null;
}

