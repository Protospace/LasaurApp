
// module to handle design data
// converts between boundry representation and gcode
// creates previews



DataHandler = {

  paths_by_color : {},
  passes : [],
  canvas : undefined,


  clear : function() {
    this.paths_by_color = {};
    passes = [];
  },

  isEmpty : function() {
    return (Object.keys(this.paths_by_color).length == 0);
  },




  // readers //////////////////////////////////

  setByPaths : function(paths_by_color) {
    // read boundaries
    // {'#000000':[[[x,y],[x,y], ..],[], ..], '#ffffff':[..]}
    this.clear();
    for (var color in paths_by_color) {
      var paths_src = paths_by_color[color];
      this.paths_by_color[color] = [];
      var paths = this.paths_by_color[color];
      for (var i=0; i<paths_src.length; i++) {
        var path = [];
        paths.push(path);
        var path_src = paths_src[i];
        for (var p=0; p<path_src.length; p++) {
          path.push([path_src[p][0], path_src[p][1]]);
        }
      }
    }

  },

  setByGcode : function(gcode) {
    // Read limited Gcode
    // G0, G00, G1, G01, G4, G04
    // G90, G91 (absolute, relative)
    // S, F, P
    // M0, M2, M3, M4, M5, M6
    // M80, M81, M82, M83, M84, M85

  },

  setByJson : function(strdata) {
    // read internal format
    // {'passes':{'colors':['#000000',..], 'feedrate':450, 'intensity':100},
    //  'paths_by_color':{'#000000':[[[x,y],[x,y], ..],[], ..], '#ffffff':[..]}
    // }
    this.clear();
    var data = JSON.parse(strdata);
    this.passes = data['passes'];
    this.paths_by_color = data['paths_by_color'];
  },



  // writers //////////////////////////////////

  getJson : function(exclude_colors) {
    // write internal format
    // exclude_colors is optional
    var paths_by_color = this.paths_by_color;
    if (!(exclude_colors === undefined)) {
      paths_by_color = {};
      for (var color in this.paths_by_color) {
        if (!(color in exclude_colors)) {
          paths_by_color[color] = this.paths_by_color[color];
        }
      }
    }
    var data = {'passes': this.passes,
                'paths_by_color': paths_by_color}
    return JSON.stringify(data);
  },

  getGcode : function() {
    // write machinable gcode, organize by passes
    // header
    var glist = [];
    glist.push("G90\nM80\n");
    glist.push("G0F"+app_settings.max_seek_speed+"\n");
    // passes
    for (var i=0; i<this.passes.length; i++) {
      var pass = this.passes[i];
      var colors = pass['colors'];
      var feedrate = this.mapConstrainFeedrate(pass['feedrate']);
      var intensity = this.mapConstrainIntesity(pass['intensity']);
      glist.push("G1F"+feedrate+"\nS"+intensity+"\n");
      for (var c=0; c<colors.length; c++) {
        var color = colors[c];
        var paths = this.paths_by_color[color];
        for (var k=0; k<paths.length; k++) {
          var path = paths[k];
          if (path.length > 0) {
            var vertex = 0;
            var x = path[vertex][0];
            var y = path[vertex][1];
            glist.push("G0X"+x.toFixed(app_settings.num_digits)+
                         "Y"+y.toFixed(app_settings.num_digits)+"\n");
            for (vertex=1; vertex<path.length; vertex++) {
              var x = path[vertex][0];
              var y = path[vertex][1];
              glist.push("G1X"+x.toFixed(app_settings.num_digits)+
                           "Y"+y.toFixed(app_settings.num_digits)+"\n");
            }
          }      
        }
      }
    }
    // footer
    glist.push("M81\nS0\nG0X0Y0F"+app_settings.max_seek_speed+"\n");
    // alert(JSON.stringify(glist.join('')))
    return glist.join('');
  },

  getBboxGcode : function() {
    // calculate bbox
    var bbox = [Infinity, Infinity, 0.0, 0.0];  // [minx, miny, maxx, maxy]
    for (var color in this.getPassesColors()) {
      var paths = this.paths_by_color[color];
      for (var k=0; k<paths.length; k++) {
        var path = paths[k];
        for (vertex=0; vertex<path.length; vertex++) {
          var x = path[vertex][0];
          var y = path[vertex][1];
          // expand bbox
          if (x < bbox[0]) {bbox[0] = x;}
          else if (x > bbox[2]) {bbox[2] = x;}
          if (y < bbox[1]) {bbox[1] = y;}
          else if (y > bbox[3]) {bbox[3] = y;}
        }
      }
    }

    var glist = [];
    glist.push("G90\n");
    glist.push("G0F"+app_settings.max_seek_speed+"\n");
    glist.push("G00X"+bbox[0].toFixed(3)+"Y"+bbox[1].toFixed(3)+"\n");
    glist.push("G00X"+bbox[2].toFixed(3)+"Y"+bbox[1].toFixed(3)+"\n");
    glist.push("G00X"+bbox[2].toFixed(3)+"Y"+bbox[3].toFixed(3)+"\n");
    glist.push("G00X"+bbox[0].toFixed(3)+"Y"+bbox[3].toFixed(3)+"\n");
    glist.push("G00X"+bbox[0].toFixed(3)+"Y"+bbox[1].toFixed(3)+"\n");
    glist.push("G0X0Y0F"+app_settings.max_seek_speed+"\n");
    return glist.join('');
  },



  // rendering //////////////////////////////////


  draw : function (canvas, scale, exclude_colors) { 
    // draw any path used in passes
    // exclude_colors is optional
    canvas.background('#ffffff');
    canvas.noFill();
    var x_prev = 0;
    var y_prev = 0;
    for (var color in this.paths_by_color) {
      if (exclude_colors === undefined || !(color in exclude_colors)) {
        var paths = this.paths_by_color[color];
        for (var k=0; k<paths.length; k++) {
          var path = paths[k];
          if (path.length > 0) {
            var x = path[0][0]*scale;
            var y = path[0][1]*scale;
            canvas.stroke('#aaaaaa');
            canvas.line(x_prev, y_prev, x, y);
            x_prev = x;
            y_prev = y;
            canvas.stroke(color);
            for (vertex=1; vertex<path.length; vertex++) {
              var x = path[vertex][0]*scale;
              var y = path[vertex][1]*scale;
              canvas.line(x_prev, y_prev, x, y);
              x_prev = x;
              y_prev = y;
            }
          }
        }
      }
    }
  },


  // passes and colors //////////////////////////

  addPass : function(mapping) {
    // this describes in what order colors are written
    // and also what intensity and feedrate is used
    // mapping: {'colors':colors, 'feedrate':feedrate, 'intensity':intensity}
    this.passes.push(mapping);
  },

  setPassesFromLasertags : function(lasertags) {
    // lasertags come in this format
    // (pass_num, feedrate, units, intensity, units, color1, color2, ..., color6)
    // [(12, 2550, '', 100, '%', ':#fff000', ':#ababab', ':#ccc999', '', '', ''), ...]
    this.passes = [];
    for (var i=0; i<lasertags.length; i++) {
      var vals = lasertags[i];
      if (vals.length == 11) {
        var pass = vals[0];
        var feedrate = vals[1];
        var intensity = vals[3];
        if (typeof(pass) === 'number' && pass > 0) {
          //make sure to have enough pass widgets
          var passes_to_create = pass - this.passes.length
          if (passes_to_create >= 1) {
            for (var k=0; k<passes_to_create; k++) {
              this.passes.push({'colors':[], 'feedrate':1200, 'intensity':10})
            }
          }
          pass = pass-1;  // convert to zero-indexed
          // feedrate
          if (feedrate != '' && typeof(feedrate) === 'number') {
            this.passes[pass]['feedrate'] = feedrate;
          }
          // intensity
          if (intensity != '' && typeof(intensity) === 'number') {
            this.passes[pass]['intensity'] = intensity;
          }
          // colors
          for (var ii=5; ii<vals.length; ii++) {
            var col = vals[ii];
            this.passes[pass]['colors'].push(col);
          }
        } else {
          $().uxmessage('error', "invalid lasertag (pass number)");
        }
      } else {
        $().uxmessage('error', "invalid lasertag (num of args)");
      }
    }
  },

  getPasses : function() {
    return this.passes;
  },

  hasPasses : function() {
    if (this.passes.length > 0) {return true}
    else {return false}
  },

  clearPasses : function() {
    this.passes = [];
  },

  getPassesColors : function() {
    var all_colors = {};
    for (var i=0; i<this.passes.length; i++) {
      var mapping = this.passes[i];
      var colors = mapping['colors'];
      for (var c=0; c<colors.length; c++) {
        var color = colors[c];
        all_colors[color] = true;
      }
    }
    return all_colors;
  },

  getAllColors : function() {
    // return list of colors
    return Object.keys(this.paths_by_color);
  },

  getColorOrder : function() {
      var color_order = {};
      var color_count = 0;
      for (var color in this.paths_by_color) {    
        color_order[color] = color_count;
        color_count++;
      }
      return color_order
  },


  // auxilliary /////////////////////////////////

  mapConstrainFeedrate : function(rate) {
    rate = parseInt(rate);
    if (rate < .1) {
      rate = .1;
      $().uxmessage('warning', "Feedrate constrained to 0.1");
    } else if (rate > 24000) {
      rate = 24000;
      $().uxmessage('warning', "Feedrate constrained to 24000");
    }
    return rate.toString();
  },
    
  mapConstrainIntesity : function(intens) {
    intens = parseInt(intens);
    if (intens < 0) {
      intens = 0;
      $().uxmessage('warning', "Intensity constrained to 0");
    } else if (intens > 100) {
      intens = 100;
      $().uxmessage('warning', "Intensity constrained to 100");
    }
    //map to 255 for now until we change the backend
    return Math.round(intens * 2.55).toString();
  },

}