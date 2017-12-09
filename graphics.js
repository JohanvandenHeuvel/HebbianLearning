/* Interactive elements
 * Model script
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

showGraphicsWarnings = true;
GraphicsConstants = {
  RESIZE_AREA: 5,
  ACTION_NONE: 0,
  ACTION_DRAG: 1,

  ACTION_E_RESIZE: 2,
  ACTION_W_RESIZE: 4,
  ACTION_S_RESIZE: 8,
  ACTION_N_RESIZE: 16,

  ACTION_NE_RESIZE: 18,
  ACTION_NW_RESIZE: 20,
  ACTION_SE_RESIZE: 10,
  ACTION_SW_RESIZE: 12
};

function ImageElement(img) {
  this.image = img;
  this.x = 0;
  this.y = 0;
  this.rotation = 0;
  this.getWidth = function() {
    return this.image.width;
  };
  this.getHeight = function() {
    return this.image.height;
  };
  this.draw = function(ctx, x = this.x, y = this.y) {
    var centerX, centerY;
    centerX = Math.floor(this.image.width/2);
    centerY = Math.floor(this.image.height/2);
    ctx.save();
    ctx.translate(x + centerX, y + centerY);
    ctx.rotate(this.rotation);
    ctx.translate(-centerX, -centerY);
    ctx.drawImage(this.image, 0, 0);
    ctx.restore();
  };
}

function makeInteractiveElement(graphicsElem, container) {
  graphicsElem.repertoire = GraphicsConstants.ACTION_DRAG;
  graphicsElem.isDraggable = function() {
    return (this.repertoire & GraphicsConstants.ACTION_DRAG) > 0;
  };
  graphicsElem.isResizable = function() {
    return (this.repertoire & GraphicsConstants.ACTION_RESIZE) > 0;
  };
  graphicsElem.currentAction = GraphicsConstants.ACTION_NONE;
  /*  graphicsElem.onResize = null;
      graphicsElem.onDrag = null;
      graphicsElem.onDragging = null;
      graphicsElem.onDrop = null;*/
  graphicsElem.startAction = function(event) {
    this.reference = {x: event.target.mouse.x, y: event.target.mouse.y};
    this.originalDimensions = {x: this.x,
                               y: this.y,
                               width: this.getWidth(),
                               height: this.getHeight()};
    this.currentAction = this.getLocalAction(event.target.mouse.x - this.x,
                                             event.target.mouse.y - this.y);
    switch (this.currentAction) {
    case GraphicsConstants.ACTION_DRAG:
      if (this.onDrag && !this.onDrag(this, event)) {
        this.currentAction = GraphicsConstants.ACTION_NONE;
      }
      break;
    case GraphicsConstants.ACTION_SE_RESIZE:
    case GraphicsConstants.ACTION_SW_RESIZE:
    case GraphicsConstants.ACTION_S_RESIZE:
    case GraphicsConstants.ACTION_NW_RESIZE:
    case GraphicsConstants.ACTION_NE_RESIZE:
    case GraphicsConstants.ACTION_W_RESIZE:
    case GraphicsConstants.ACTION_S_RESIZE:
    case GraphicsConstants.ACTION_N_RESIZE:
    case GraphicsConstants.ACTION_E_RESIZE:
      if (this.onResize && !this.onResize(this, event, this.currentAction)) {
        this.currentAction = GraphicsConstants.ACTION_NONE;
      }
      break;
    }
  };
  graphicsElem.endAction = function(event) {
    if (this.currentAction == GraphicsConstants.ACTION_DRAG && this.onDrop) {
      this.onDrop(this, event);
    }
    this.currentAction = GraphicsConstants.ACTION_NONE;
  };
  graphicsElem.continueAction = function(event) {
    var x, y;
    x = event.target.mouse.x;
    y = event.target.mouse.y;
    if (this.constraints != null) {
      x = Math.min(this.constraints.right + this.reference.x,
                   Math.max(this.constraints.left + this.reference.x, x));
      y = Math.min(this.constraints.bottom + this.reference.y,
                   Math.max(this.constraints.top + this.reference.y, y));
    }
    if ((this.currentAction & GraphicsConstants.ACTION_DRAG) > 0) {
      this.x = this.originalDimensions.x + x - this.reference.x;
      this.y = this.originalDimensions.y + y - this.reference.y;
    } else {
      if ((this.currentAction & GraphicsConstants.ACTION_S_RESIZE) > 0) {
        this.elem.setHeight(this.originalDimensions.height + y - this.reference.y);
      }
      if ((this.currentAction & GraphicsConstants.ACTION_E_RESIZE) > 0) {
        this.elem.setWidth(this.originalDimensions.width + x - this.reference.x);
      }
      if ((this.currentAction & GraphicsConstants.ACTION_N_RESIZE) > 0) {
        this.y = this.originalDimensions.y + y - this.reference.y;
        this.elem.setHeight(this.originalDimensions.height - y + this.reference.y);
      }
      if ((this.currentAction & GraphicsConstants.ACTION_W_RESIZE) > 0) {
        this.x = this.originalDimensions.x + x - this.reference.x;
        this.elem.setWidth(this.originalDimensions.width - x + this.reference.x);
      }
    }
    if (this.onDragging) {
      this.onDragging(this, event.target);
    }
  };
  graphicsElem.getActionLabel = function(x, y) {
    switch (this.getLocalAction(x,y)) {
    case GraphicsConstants.ACTION_W_RESIZE:
    case GraphicsConstants.ACTION_E_RESIZE:
      return "col-resize";
    case GraphicsConstants.ACTION_S_RESIZE:
    case GraphicsConstants.ACTION_N_RESIZE:
      return "row-resize";
    case GraphicsConstants.ACTION_NW_RESIZE:
      return "nw-resize";
    case GraphicsConstants.ACTION_NE_RESIZE:
      return "ne-resize";
    case GraphicsConstants.ACTION_SE_RESIZE:
      return "se-resize";
    case GraphicsConstants.ACTION_SW_RESIZE:
      return "sw-resize";
    case GraphicsConstants.ACTION_DRAG:
      return "pointer";
    default:
      return undefined;
    }
  };

  graphicsElem.getLocalAction = function(x, y) {
    if ((this.repertoire & GraphicsConstants.ACTION_W_RESIZE) > 0 &&
        x < GraphicsConstants.RESIZE_AREA) {
      if ((this.repertoire & GraphicsConstants.ACTION_N_RESIZE) > 0 &&
          y < GraphicsConstants.RESIZE_AREA) {
        return GraphicsConstants.ACTION_NW_RESIZE;
      } else if ((this.repertoire & GraphicsConstants.ACTION_S_RESIZE) > 0 &&
                 y > this.getHeight() - GraphicsConstants.RESIZE_AREA) {
        return GraphicsConstants.ACTION_SW_RESIZE;
      }
      return GraphicsConstants.ACTION_W_RESIZE;
    } else if ((this.repertoire & GraphicsConstants.ACTION_E_RESIZE) > 0 &&
               x > this.getWidth() - GraphicsConstants.RESIZE_AREA) {
      if ((this.repertoire & GraphicsConstants.ACTION_N_RESIZE) > 0 &&
          y < GraphicsConstants.RESIZE_AREA) {
        return GraphicsConstants.ACTION_NE_RESIZE;
      } else if ((this.repertoire & GraphicsConstants.ACTION_S_RESIZE) > 0 &&
                 y > this.getHeight() - GraphicsConstants.RESIZE_AREA) {
        return GraphicsConstants.ACTION_SE_RESIZE;
      }
      return GraphicsConstants.ACTION_E_RESIZE;
    } else if ((this.repertoire & GraphicsConstants.ACTION_N_RESIZE) > 0 &&
               y < GraphicsConstants.RESIZE_AREA) {
      return GraphicsConstants.ACTION_N_RESIZE;
    } else if ((this.repertoire & GraphicsConstants.ACTION_S_RESIZE) > 0 &&
               y > this.getHeight() - GraphicsConstants.RESIZE_AREA) {
      return GraphicsConstants.ACTION_S_RESIZE;
    }
    if (this.isDraggable) {
      return GraphicsConstants.ACTION_DRAG;
    }
    return undefined;
  };

  if (container.addInteractiveElement) {
    container.addInteractiveElement(graphicsElem);
  } else if (showGraphicsWarnings) {
    console.warn("Container does not support mouse tracking: element will not be interactive.");
  }
  return graphicsElem;
}

