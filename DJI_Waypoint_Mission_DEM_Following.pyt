# -*- coding: utf-8 -*-
"""
DJI Mini 4 Pro Waypoint Mission Generator  v5.0
ArcGIS Pro Python Toolbox (.pyt)

New in v5:
  - Mission Name (embedded in KMZ metadata)
  - Front Overlap % and Side Overlap % dropdowns (auto-calculate spacing)
  - Edge Buffer (inset polygon before grid generation)
  - Hover Time at Waypoints (motion blur reduction)
  - AE Lock at First Waypoint (extended initial hover for camera settling)
  - Imperial Units toggle (feet input, converted to metres internally)

Output: DJI WPML 1.0.2 .kmz
Drone:  DJI Mini 4 Pro (droneEnumValue = 67)  +  RC Pro / DJI Pilot 2
"""

import arcpy
import math
import zipfile
import os
import json
import urllib.request
import urllib.error
from datetime import datetime


# Conversion factors
FT_PER_M       = 3.28084
M_PER_FT       = 1.0 / FT_PER_M
HA_PER_M2      = 1.0 / 10000.0
ACRE_PER_M2    = 0.000247105


class Toolbox(object):
    def __init__(self):
        self.label = "DJI Waypoint Mission Generator"
        self.alias = "DJIWaypointMission"
        self.tools = [GenerateWaypointMission]


class GenerateWaypointMission(object):
    # ------------------------------------------------------------------
    # DJI Mini 4 Pro camera constants
    # ------------------------------------------------------------------
    FOCAL_MM    = 9.74
    SENSOR_W_MM = 9.60
    IMG_W_PX    = 8064
    HFOV_DEG    = 82.1   # horizontal field of view
    VFOV_DEG    = 65.2   # vertical field of view

    OVERLAP_OPTIONS = ["Manual", "60%", "65%", "70%", "75%", "80%", "85%", "90%"]

    def __init__(self):
        self.label = "Generate DJI Waypoint Mission (Mini 4 Pro)"
        self.description = (
            "Creates a DJI WPML (.kmz) waypoint mission from a polygon. "
            "Supports overlap %, edge buffer, hover time, AE lock, "
            "auto-detected strip direction, terrain following, and imperial units."
        )
        self.canRunInBackground = False

    # ------------------------------------------------------------------
    def getParameterInfo(self):
        params = []

        # 0 - Input polygon
        p = arcpy.Parameter(displayName="Input Polygon Feature",
                            name="input_feature", datatype="GPFeatureLayer",
                            parameterType="Required", direction="Input")
        params.append(p)

        # 1 - Output KMZ
        p = arcpy.Parameter(displayName="Output KMZ File",
                            name="output_kmz", datatype="DEFile",
                            parameterType="Required", direction="Output")
        p.filter.list = ["kmz"]
        params.append(p)

        # 2 - Imperial units (moved to top for UX)
        p = arcpy.Parameter(
            displayName=(
                "Imperial Units - treat altitude, spacing, interval, and edge buffer as feet"
            ),
            name="imperial_units", datatype="GPBoolean",
            parameterType="Optional", direction="Input")
        p.value = False
        params.append(p)

        # 3 - Mission Name
        p = arcpy.Parameter(
            displayName="Mission Name (embedded in KMZ metadata)",
            name="mission_name", datatype="GPString",
            parameterType="Optional", direction="Input")
        p.value = ""
        params.append(p)

        # 5 - Altitude
        p = arcpy.Parameter(
            displayName="Flight Altitude AGL (units depend on Imperial toggle above)",
            name="altitude", datatype="GPDouble",
            parameterType="Required", direction="Input")
        p.value = 60.0
        params.append(p)

        # 5 - Speed (always m/s - WPML native)
        p = arcpy.Parameter(displayName="Flight Speed (m/s,  max 15)",
                            name="speed", datatype="GPDouble",
                            parameterType="Required", direction="Input")
        p.value = 8.0
        params.append(p)

        # 6 - Front overlap
        p = arcpy.Parameter(
            displayName="Front Overlap % (along-track) - select to auto-calculate Photo Interval",
            name="front_overlap", datatype="GPString",
            parameterType="Required", direction="Input")
        p.filter.type = "ValueList"
        p.filter.list = self.OVERLAP_OPTIONS
        p.value = "Manual"
        params.append(p)

        # 7 - Side overlap
        p = arcpy.Parameter(
            displayName="Side Overlap % (cross-track) - select to auto-calculate Strip Spacing",
            name="side_overlap", datatype="GPString",
            parameterType="Required", direction="Input")
        p.filter.type = "ValueList"
        p.filter.list = self.OVERLAP_OPTIONS
        p.value = "Manual"
        params.append(p)

        # 8 - Strip Spacing
        p = arcpy.Parameter(
            displayName="Strip Spacing - distance between parallel passes",
            name="strip_spacing", datatype="GPDouble",
            parameterType="Required", direction="Input")
        p.value = 30.0
        params.append(p)

        # 9 - Photo Interval
        p = arcpy.Parameter(
            displayName="Photo Interval - distance between photos along each pass",
            name="photo_interval", datatype="GPDouble",
            parameterType="Required", direction="Input")
        p.value = 20.0
        params.append(p)

        # 10 - Strip direction
        p = arcpy.Parameter(
            displayName="Strip Direction - bearing of each pass, degrees from North (0-360)",
            name="flight_direction", datatype="GPDouble",
            parameterType="Required", direction="Input")
        p.value = 0.0
        params.append(p)

        # 11 - Auto-detect strip direction
        p = arcpy.Parameter(
            displayName=(
                "Auto-detect Optimal Strip Direction from Polygon Long Axis "
                "(overrides Strip Direction above)"
            ),
            name="auto_strip_dir", datatype="GPBoolean",
            parameterType="Optional", direction="Input")
        p.value = False
        params.append(p)

        # 12 - Edge Buffer
        p = arcpy.Parameter(
            displayName="Edge Buffer - inset polygon inward before generating grid (0 = no buffer)",
            name="edge_buffer", datatype="GPDouble",
            parameterType="Optional", direction="Input")
        p.value = 0.0
        params.append(p)

        # 13 - Gimbal pitch
        p = arcpy.Parameter(
            displayName="Gimbal Pitch Angle (degrees)   -90 = nadir   0 = forward",
            name="gimbal_pitch", datatype="GPDouble",
            parameterType="Required", direction="Input")
        p.value = -90.0
        params.append(p)

        # 14 - Hover Time
        p = arcpy.Parameter(
            displayName="Hover Time at Each Waypoint (seconds, 0 = no hover)",
            name="hover_time", datatype="GPDouble",
            parameterType="Optional", direction="Input")
        p.value = 0.0
        params.append(p)

        # 15 - AE Lock
        p = arcpy.Parameter(
            displayName=(
                "AE Lock at First Waypoint "
                "(adds 3-second initial hover for camera exposure to settle)"
            ),
            name="ae_lock", datatype="GPBoolean",
            parameterType="Optional", direction="Input")
        p.value = False
        params.append(p)

        # 16 - Heading mode
        p = arcpy.Parameter(displayName="Drone Heading Mode",
                            name="heading_mode", datatype="GPString",
                            parameterType="Required", direction="Input")
        p.filter.type = "ValueList"
        p.filter.list = ["followWayline", "fixed", "smoothTransition"]
        p.value = "fixed"
        params.append(p)

        # 17 - Finish action
        p = arcpy.Parameter(displayName="Mission Finish Action",
                            name="finish_action", datatype="GPString",
                            parameterType="Required", direction="Input")
        p.filter.type = "ValueList"
        p.filter.list = ["goHome", "autoLand", "gotoFirstWaypoint"]
        p.value = "goHome"
        params.append(p)

        # 18 - RC lost
        p = arcpy.Parameter(displayName="RC Signal Lost Action",
                            name="rc_lost_action", datatype="GPString",
                            parameterType="Required", direction="Input")
        p.filter.type = "ValueList"
        p.filter.list = ["goBack", "hover", "land", "goContinue"]
        p.value = "goBack"
        params.append(p)

        # 19 - Double grid
        p = arcpy.Parameter(
            displayName="Double Grid - add cross-hatch pass at 90 degrees",
            name="double_grid", datatype="GPBoolean",
            parameterType="Optional", direction="Input")
        p.value = False
        params.append(p)

        # 20 - Reverse mission
        p = arcpy.Parameter(
            displayName="Reverse Mission - flip waypoint order",
            name="reverse_mission", datatype="GPBoolean",
            parameterType="Optional", direction="Input")
        p.value = False
        params.append(p)

        # 21 - Terrain follow
        p = arcpy.Parameter(
            displayName="Enable Terrain Following",
            name="terrain_follow", datatype="GPBoolean",
            parameterType="Optional", direction="Input")
        p.value = False
        params.append(p)

        # 22 - DEM raster
        p = arcpy.Parameter(
            displayName=(
                "DEM Raster (optional - leave blank to use Open-Meteo API at 90 m resolution)"
            ),
            name="dem_raster",
            datatype="GPRasterLayer",
            parameterType="Optional",
            direction="Input")
        p.value = None
        params.append(p)

        # 23 - Add preview
        p = arcpy.Parameter(
            displayName="Add Preview Layers to Active Map",
            name="add_preview", datatype="GPBoolean",
            parameterType="Optional", direction="Input")
        p.value = True
        params.append(p)

        return params

    # ------------------------------------------------------------------
    def isLicensed(self):
        return True

    def updateParameters(self, parameters):
        # Front overlap % overrides photo interval
        if parameters[6].value and parameters[6].value != "Manual":
            parameters[9].enabled = False
        else:
            parameters[9].enabled = True

        # Side overlap % overrides strip spacing
        if parameters[7].value and parameters[7].value != "Manual":
            parameters[8].enabled = False
        else:
            parameters[8].enabled = True

        # Auto-detect overrides manual strip direction
        parameters[10].enabled = not bool(parameters[11].value)

        # DEM only enabled when terrain following is on
        parameters[22].enabled = bool(parameters[21].value)

        # Update unit labels in displayName based on imperial toggle
        # (ArcGIS doesn't allow dynamic display name changes after init,
        #  so we rely on the imperial toggle's description for clarity)

    def updateMessages(self, parameters):
        alt   = parameters[4].value
        speed = parameters[5].value
        ss    = parameters[8].value
        pi    = parameters[9].value
        eb    = parameters[12].value
        gp    = parameters[13].value
        hov   = parameters[14].value

        if alt is not None:
            # Altitude check (account for imperial)
            alt_m = alt * M_PER_FT if parameters[2].value else alt
            if alt_m < 2:
                parameters[4].setWarningMessage("Altitude below 2 m.")
            elif alt_m > 120:
                parameters[4].setWarningMessage(
                    "Altitude > 120 m AGL (~393 ft) requires Transport Canada "
                    "Advanced Operations certificate or SFOC.")

        if speed is not None and speed > 15:
            parameters[5].setErrorMessage("Maximum waypoint speed is 15 m/s.")

        if ss is not None and ss < 1:
            parameters[8].setErrorMessage("Strip spacing must be >= 1 unit.")

        if pi is not None and pi < 1:
            parameters[9].setErrorMessage("Photo interval must be >= 1 unit.")

        if eb is not None and eb < 0:
            parameters[12].setErrorMessage("Edge buffer must be >= 0.")

        if gp is not None and not (-90 <= gp <= 30):
            parameters[13].setWarningMessage("Typical range: -90 (nadir) to 0 (forward).")

        if hov is not None and (hov < 0 or hov > 30):
            parameters[14].setWarningMessage("Hover time should be 0-30 seconds.")

        if parameters[21].value and not parameters[22].value:
            parameters[21].setWarningMessage(
                "No DEM provided - will query Open-Meteo API "
                "(internet required, 90 m res).")

    # ==========================================================================
    # execute
    # ==========================================================================
    def execute(self, parameters, messages):
        input_feature   = parameters[0].valueAsText
        output_kmz      = parameters[1].valueAsText
        mission_name    = (parameters[3].valueAsText or "").strip()
        altitude_in     = float(parameters[4].value)
        speed           = float(parameters[5].value)
        front_overlap   = parameters[6].valueAsText
        side_overlap    = parameters[7].valueAsText
        strip_spacing_in = float(parameters[8].value)
        photo_interval_in = float(parameters[9].value)
        flight_dir_deg  = float(parameters[10].value)
        auto_strip_dir  = bool(parameters[11].value)
        edge_buffer_in  = float(parameters[12].value or 0.0)
        gimbal_pitch    = float(parameters[13].value)
        hover_time      = float(parameters[14].value or 0.0)
        ae_lock         = bool(parameters[15].value)
        heading_mode    = parameters[16].valueAsText
        finish_action   = parameters[17].valueAsText
        rc_lost_action  = parameters[18].valueAsText
        double_grid     = bool(parameters[19].value)
        reverse_mis     = bool(parameters[20].value)
        terrain_follow  = bool(parameters[21].value)
        dem_path        = parameters[22].valueAsText if parameters[22].value else None
        imperial        = bool(parameters[2].value)
        add_preview     = bool(parameters[23].value)

        # ---- Convert imperial inputs to metric for processing ----
        if imperial:
            altitude       = altitude_in       * M_PER_FT
            strip_spacing  = strip_spacing_in  * M_PER_FT
            photo_interval = photo_interval_in * M_PER_FT
            edge_buffer    = edge_buffer_in    * M_PER_FT
            messages.addMessage(
                "Imperial inputs converted to metric: alt={:.1f} m, "
                "spacing={:.1f} m, interval={:.1f} m, buffer={:.1f} m".format(
                    altitude, strip_spacing, photo_interval, edge_buffer))
        else:
            altitude       = altitude_in
            strip_spacing  = strip_spacing_in
            photo_interval = photo_interval_in
            edge_buffer    = edge_buffer_in

        if not output_kmz.lower().endswith(".kmz"):
            output_kmz += ".kmz"

        sr_albers = arcpy.SpatialReference(3005)
        sr_wgs84  = arcpy.SpatialReference(4326)

        desc = arcpy.Describe(input_feature)
        if desc.shapeType != "Polygon":
            messages.addErrorMessage(
                "Input must be a Polygon layer. Got: {}".format(desc.shapeType))
            return

        # ---- Apply overlap % to spacing/interval if selected ----------
        fp_w_m = 2.0 * altitude * math.tan(math.radians(self.HFOV_DEG / 2.0))
        fp_h_m = 2.0 * altitude * math.tan(math.radians(self.VFOV_DEG / 2.0))

        messages.addMessage(
            "Camera footprint at {:.1f} m AGL: {:.1f} m wide x {:.1f} m deep".format(
                altitude, fp_w_m, fp_h_m))

        if front_overlap and front_overlap != "Manual":
            pct = float(front_overlap.rstrip("%")) / 100.0
            photo_interval = fp_h_m * (1.0 - pct)
            messages.addMessage(
                "Front overlap {} -> photo interval = {:.2f} m".format(
                    front_overlap, photo_interval))

        if side_overlap and side_overlap != "Manual":
            pct = float(side_overlap.rstrip("%")) / 100.0
            strip_spacing = fp_w_m * (1.0 - pct)
            messages.addMessage(
                "Side overlap {} -> strip spacing = {:.2f} m".format(
                    side_overlap, strip_spacing))

        # ---- GSD info -------------------------------------------------
        gsd = altitude * self.SENSOR_W_MM / (self.FOCAL_MM * self.IMG_W_PX) * 100
        messages.addMessage(
            "Mini 4 Pro GSD at {:.1f} m AGL: {:.2f} cm/pixel".format(altitude, gsd))

        # ---- Apply edge buffer ----------------------------------------
        # Buffer the input polygon inward by edge_buffer metres
        working_layer = input_feature
        buffered_path = None

        if edge_buffer > 0.001:
            buffered_path = "in_memory/dji_buffered_polygon"
            if arcpy.Exists(buffered_path):
                arcpy.management.Delete(buffered_path)

            messages.addMessage(
                "Applying edge buffer: -{:.1f} m inset...".format(edge_buffer))
            try:
                arcpy.analysis.Buffer(
                    in_features=input_feature,
                    out_feature_class=buffered_path,
                    buffer_distance_or_field="-{} Meters".format(edge_buffer))
                # Verify buffer didn't eliminate the polygon
                count = int(arcpy.management.GetCount(buffered_path).getOutput(0))
                if count == 0:
                    messages.addErrorMessage(
                        "Edge buffer of {:.1f} m eliminated the polygon entirely. "
                        "Reduce the buffer distance.".format(edge_buffer))
                    return
                working_layer = buffered_path
            except Exception as e:
                messages.addWarningMessage(
                    "Edge buffer failed: {}. Using original polygon.".format(e))
                working_layer = input_feature

        # ---- Compute survey area --------------------------------------
        try:
            survey_area_m2 = 0.0
            with arcpy.da.SearchCursor(working_layer, ["SHAPE@"]) as cursor:
                for (geom,) in cursor:
                    survey_area_m2 += geom.projectAs(sr_albers).area
            messages.addMessage(
                "Survey area: {:.2f} ha  ({:.2f} acres)".format(
                    survey_area_m2 * HA_PER_M2,
                    survey_area_m2 * ACRE_PER_M2))
        except Exception as e:
            messages.addWarningMessage(
                "Could not calculate area: {}".format(e))
            survey_area_m2 = 0.0

        # ---- Auto-detect strip direction ------------------------------
        if auto_strip_dir:
            with arcpy.da.SearchCursor(working_layer, ["SHAPE@"]) as cursor:
                for (geom,) in cursor:
                    geom_albers = geom.projectAs(sr_albers)
                    detected = self._auto_strip_direction(geom_albers, messages)
                    if detected is not None:
                        flight_dir_deg = detected
                    break

        # ---- Primary grid ---------------------------------------------
        all_waypoints  = []
        all_strip_nums = []
        all_grid_types = []

        with arcpy.da.SearchCursor(working_layer, ["SHAPE@", "OID@"]) as cursor:
            for geom, oid in cursor:
                messages.addMessage("Processing feature OID {}...".format(oid))
                geom_albers = geom.projectAs(sr_albers)
                wpts, snums = self._lawnmower(
                    geom_albers, strip_spacing, photo_interval,
                    flight_dir_deg, sr_albers, sr_wgs84, messages)
                all_waypoints.extend(wpts)
                all_strip_nums.extend(snums)
                all_grid_types.extend(["Primary"] * len(wpts))

        # ---- Double grid -----------------------------------------------
        if double_grid and all_waypoints:
            messages.addMessage("Generating cross-hatch grid (+90 deg)...")
            second_dir   = (flight_dir_deg + 90.0) % 360.0
            strip_offset = (max(all_strip_nums) + 1) if all_strip_nums else 0
            with arcpy.da.SearchCursor(working_layer, ["SHAPE@"]) as cursor:
                for (geom,) in cursor:
                    geom_albers = geom.projectAs(sr_albers)
                    wpts2, snums2 = self._lawnmower(
                        geom_albers, strip_spacing, photo_interval,
                        second_dir, sr_albers, sr_wgs84, messages)
                    all_waypoints.extend(wpts2)
                    all_strip_nums.extend([s + strip_offset for s in snums2])
                    all_grid_types.extend(["Cross-hatch"] * len(wpts2))

        # Cleanup buffered polygon
        if buffered_path and arcpy.Exists(buffered_path):
            arcpy.management.Delete(buffered_path)

        if not all_waypoints:
            messages.addErrorMessage(
                "No waypoints generated. Polygon may be too small for the spacing, "
                "or the edge buffer may have eliminated viable area.")
            return

        # ---- Terrain following ----------------------------------------
        altitudes      = [altitude] * len(all_waypoints)
        home_elev      = None
        raw_wp_elevs   = None
        terrain_source = None

        if terrain_follow:
            query_pts = [all_waypoints[0]] + all_waypoints
            if dem_path:
                messages.addMessage("Terrain following: sampling local DEM raster...")
                elevs = self._sample_dem(query_pts, dem_path, sr_wgs84, messages)
                terrain_source = "Local DEM"
            else:
                messages.addMessage(
                    "Terrain following: querying Open-Meteo API (Copernicus GLO-90)...")
                elevs = self._fetch_open_meteo(query_pts, messages)
                terrain_source = "Open-Meteo API (90 m)"

            if elevs and elevs[0] is not None:
                home_elev    = elevs[0]
                raw_wp_elevs = elevs[1:]
                valid_elevs  = [e for e in raw_wp_elevs if e is not None]
                none_count   = len(raw_wp_elevs) - len(valid_elevs)

                if valid_elevs:
                    messages.addMessage("Source       : {}".format(terrain_source))
                    messages.addMessage(
                        "Home elevation: {:.1f} m ASL".format(home_elev))
                    messages.addMessage(
                        "Terrain range : {:.1f} - {:.1f} m ASL  "
                        "(relief: {:.1f} m)".format(
                            min(valid_elevs), max(valid_elevs),
                            max(valid_elevs) - min(valid_elevs)))
                    for i, wp_elev in enumerate(raw_wp_elevs):
                        if wp_elev is not None:
                            altitudes[i] = max(10.0, altitude + (wp_elev - home_elev))
                    messages.addMessage(
                        "Adjusted height range above takeoff: "
                        "{:.1f} - {:.1f} m".format(min(altitudes), max(altitudes)))
                if none_count > 0:
                    messages.addWarningMessage(
                        "{} waypoints had no elevation data.".format(none_count))
            else:
                messages.addWarningMessage(
                    "Could not retrieve terrain data. Flat altitude will be used.")

        # ---- Reverse ----------------------------------------------------
        if reverse_mis:
            all_waypoints  = list(reversed(all_waypoints))
            altitudes      = list(reversed(altitudes))
            all_strip_nums = list(reversed(all_strip_nums))
            all_grid_types = list(reversed(all_grid_types))
            if raw_wp_elevs is not None:
                raw_wp_elevs = list(reversed(raw_wp_elevs))
            messages.addMessage("Mission order reversed.")

        # ---- Validate ---------------------------------------------------
        n        = len(all_waypoints)
        est_dist = self._total_distance(all_waypoints)
        # Hover time adds to total time per waypoint
        ae_lock_extra = 3.0 if ae_lock else 0.0
        est_time = (est_dist / speed + n * hover_time + ae_lock_extra) / 60.0

        messages.addMessage("\n--- Mission Summary ---")
        if mission_name:
            messages.addMessage("Mission name : {}".format(mission_name))
        messages.addMessage("Waypoints    : {}".format(n))
        messages.addMessage("Strips       : {}".format(
            max(all_strip_nums) + 1 if all_strip_nums else 0))
        messages.addMessage("GSD          : {:.2f} cm/pixel".format(gsd))
        messages.addMessage(
            "Est. distance: {:.0f} m  ({:.0f} ft)".format(
                est_dist, est_dist * FT_PER_M))
        messages.addMessage(
            "Est. flight time: {:.1f} min at {} m/s".format(est_time, speed))
        if hover_time > 0:
            messages.addMessage(
                "  (includes {} s hover at each of {} waypoints "
                "= {:.1f} min hover)".format(
                    hover_time, n, n * hover_time / 60.0))
        if ae_lock:
            messages.addMessage("  (includes 3 s AE lock hover at first waypoint)")
        if survey_area_m2 > 0:
            messages.addMessage(
                "Survey area  : {:.2f} ha  ({:.2f} acres)".format(
                    survey_area_m2 * HA_PER_M2, survey_area_m2 * ACRE_PER_M2))

        if n > 65535:
            messages.addErrorMessage(
                "{} waypoints exceeds DJI WPML max of 65,535.".format(n))
            return
        if n > 2000:
            messages.addWarningMessage(
                "{} waypoints is large. Consider splitting.".format(n))

        # ---- Write KMZ --------------------------------------------------
        out_dir = os.path.dirname(output_kmz)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir)

        self._write_kmz(all_waypoints, altitudes, output_kmz, speed,
                        gimbal_pitch, heading_mode, finish_action, rc_lost_action,
                        hover_time, ae_lock, mission_name)
        messages.addMessage("\nKMZ written: {}".format(output_kmz))

        # ---- Map preview ------------------------------------------------
        if add_preview:
            messages.addMessage("\nBuilding preview layer...")
            preview_meta = {
                "gimbal_pitch"  : gimbal_pitch,
                "speed"         : speed,
                "heading_mode"  : heading_mode,
                "finish_action" : finish_action,
                "rc_lost_action": rc_lost_action,
                "strip_nums"    : all_strip_nums,
                "grid_types"    : all_grid_types,
                "home_elev"     : home_elev,
                "raw_wp_elevs"  : raw_wp_elevs,
                "terrain_follow": terrain_follow,
                "terrain_source": terrain_source,
                "hover_time"    : hover_time,
                "ae_lock"       : ae_lock,
                "mission_name"  : mission_name,
                "survey_area_m2": survey_area_m2,
            }
            self._add_preview_to_map(
                all_waypoints, altitudes, preview_meta, messages)

    # ==========================================================================
    # Auto-detect optimal strip direction
    # ==========================================================================
    def _auto_strip_direction(self, polygon_albers, messages):
        try:
            sr = polygon_albers.spatialReference
            temp_in  = "in_memory/dji_autodir_in"
            temp_mbr = "in_memory/dji_autodir_mbr"

            for fc in (temp_in, temp_mbr):
                if arcpy.Exists(fc):
                    arcpy.management.Delete(fc)

            arcpy.management.CreateFeatureclass(
                "in_memory", "dji_autodir_in", "POLYGON", spatial_reference=sr)
            with arcpy.da.InsertCursor(temp_in, ["SHAPE@"]) as cur:
                cur.insertRow([polygon_albers])

            arcpy.management.MinimumBoundingGeometry(
                temp_in, temp_mbr, "RECTANGLE_BY_AREA", "NONE",
                add_attributes="ADD_ATTRIBUTES")

            orientation = None
            with arcpy.da.SearchCursor(
                    temp_mbr,
                    ["MBG_Orientation", "MBG_Length", "MBG_Width"]) as cur:
                for row in cur:
                    orient, length, width = float(row[0]), float(row[1]), float(row[2])
                    orientation = orient
                    ratio = length / width if width > 0 else 0
                    messages.addMessage(
                        "Auto strip direction: long axis {:.1f} deg from North  "
                        "(long: {:.1f} m, short: {:.1f} m, ratio: {:.1f}:1)".format(
                            orient, length, width, ratio))
                    if 0 < ratio < 1.1:
                        messages.addWarningMessage(
                            "Polygon is nearly square (ratio {:.1f}:1). "
                            "Auto strip direction may not offer much advantage.".format(
                                ratio))
                    break

            for fc in (temp_in, temp_mbr):
                if arcpy.Exists(fc):
                    arcpy.management.Delete(fc)
            return orientation

        except Exception as e:
            messages.addWarningMessage(
                "Auto strip direction failed: {}".format(e))
            return None

    # ==========================================================================
    # Lawnmower grid
    # ==========================================================================
    @staticmethod
    def _rotate(x, y, cx, cy, angle_rad):
        dx, dy = x - cx, y - cy
        return (dx * math.cos(angle_rad) - dy * math.sin(angle_rad) + cx,
                dx * math.sin(angle_rad) + dy * math.cos(angle_rad) + cy)

    def _lawnmower(self, polygon_albers, strip_spacing, photo_interval,
                   flight_dir_deg, sr_albers, sr_wgs84, messages):
        extent = polygon_albers.extent
        cx = (extent.XMin + extent.XMax) / 2.0
        cy = (extent.YMin + extent.YMax) / 2.0

        math_angle   = math.radians(90.0 - flight_dir_deg)
        rot_to_grid  =  math_angle
        rot_to_world = -math_angle

        part  = polygon_albers.getPart(0)
        verts = [(pnt.X, pnt.Y) for pnt in part if pnt is not None]
        if len(verts) < 3:
            messages.addWarningMessage("Polygon has < 3 vertices - skipped.")
            return [], []
        if verts[0] != verts[-1]:
            verts.append(verts[0])

        rv    = [self._rotate(x, y, cx, cy, rot_to_grid) for x, y in verts]
        min_y = min(v[1] for v in rv)
        max_y = max(v[1] for v in rv)
        edges = list(zip(rv[:-1], rv[1:]))

        strip_ys = []
        y = min_y + strip_spacing / 2.0
        while y <= max_y + 1e-6:
            strip_ys.append(y)
            y += strip_spacing

        if not strip_ys:
            messages.addWarningMessage(
                "No strips fit - check polygon vs strip spacing.")
            return [], []

        waypoints     = []
        strip_nums    = []
        strip_counter = 0

        for strip_idx, sy in enumerate(strip_ys):
            xs = []
            for (x1, y1), (x2, y2) in edges:
                if abs(y2 - y1) < 1e-9:
                    continue
                t = (sy - y1) / (y2 - y1)
                if 0.0 <= t <= 1.0:
                    xs.append(x1 + t * (x2 - x1))
            if len(xs) < 2:
                continue
            xs.sort()

            for pi in range(0, len(xs) - 1, 2):
                x_a, x_b = xs[pi], xs[pi + 1]
                seg = abs(x_b - x_a)
                if seg < 0.1:
                    continue
                if strip_idx % 2 == 1:
                    x_a, x_b = x_b, x_a
                direction   = 1 if x_b >= x_a else -1
                seg         = abs(x_b - x_a)
                n_intervals = max(1, int(math.ceil(seg / photo_interval)))
                actual_ivl  = seg / n_intervals

                for k in range(n_intervals + 1):
                    gx = x_a + direction * k * actual_ivl
                    wx, wy = self._rotate(gx, sy, cx, cy, rot_to_world)
                    pt    = arcpy.PointGeometry(arcpy.Point(wx, wy), sr_albers)
                    pt_wg = pt.projectAs(sr_wgs84)
                    waypoints.append((pt_wg.firstPoint.X, pt_wg.firstPoint.Y))
                    strip_nums.append(strip_counter)
                strip_counter += 1

        messages.addMessage("  -> {} strips, {} waypoints (dir {:.0f} deg)".format(
            strip_counter, len(waypoints), flight_dir_deg))
        return waypoints, strip_nums

    # ==========================================================================
    # Terrain helpers
    # ==========================================================================
    def _fetch_open_meteo(self, waypoints_wgs84, messages):
        BATCH = 100
        results = []
        total   = len(waypoints_wgs84)
        n_batch = math.ceil(total / BATCH)

        for i in range(0, total, BATCH):
            batch = waypoints_wgs84[i:i + BATCH]
            lats  = ','.join('{:.6f}'.format(p[1]) for p in batch)
            lons  = ','.join('{:.6f}'.format(p[0]) for p in batch)
            url   = ('https://api.open-meteo.com/v1/elevation'
                     '?latitude={}&longitude={}'.format(lats, lons))
            if n_batch > 1:
                messages.addMessage(
                    "  API batch {}/{} ({} pts)".format(
                        i // BATCH + 1, n_batch, len(batch)))
            try:
                req = urllib.request.Request(
                    url, headers={'User-Agent': 'ArcGIS-DJI-MissionGen/5.0'})
                with urllib.request.urlopen(req, timeout=30) as resp:
                    data = json.loads(resp.read().decode('utf-8'))
                    results.extend(
                        [None if e is None else float(e)
                         for e in data.get('elevation', [])])
            except Exception as e:
                messages.addWarningMessage(
                    "API error (batch {}): {}".format(i // BATCH + 1, e))
                results.extend([None] * len(batch))
        return results

    def _sample_dem(self, waypoints_wgs84, dem_path, sr_wgs84, messages):
        try:
            raster    = arcpy.Raster(dem_path)
            raster_sr = raster.spatialReference
            extent    = raster.extent
            cw        = raster.meanCellWidth
            ch        = raster.meanCellHeight
            nodata    = raster.noDataValue

            messages.addMessage("Loading DEM array...")
            arr          = arcpy.RasterToNumPyArray(raster, nodata_to_value=-9999.0)
            nrows, ncols = arr.shape
            elevs        = []
            for lon, lat in waypoints_wgs84:
                pt   = arcpy.PointGeometry(arcpy.Point(lon, lat), sr_wgs84)
                pt_r = pt.projectAs(raster_sr)
                x, y = pt_r.firstPoint.X, pt_r.firstPoint.Y
                col  = int((x - extent.XMin) / cw)
                row  = int((extent.YMax - y) / ch)
                if 0 <= row < nrows and 0 <= col < ncols:
                    v = float(arr[row, col])
                    elevs.append(
                        None if (v == -9999.0 or
                                 (nodata is not None and abs(v - nodata) < 0.01))
                        else v)
                else:
                    elevs.append(None)
            return elevs
        except MemoryError:
            messages.addWarningMessage(
                "DEM too large for array - falling back to point sampling.")
            return self._sample_dem_point(waypoints_wgs84, dem_path, sr_wgs84, messages)
        except Exception as e:
            messages.addWarningMessage(
                "DEM error: {} - falling back to Open-Meteo API.".format(e))
            return self._fetch_open_meteo(waypoints_wgs84, messages)

    def _sample_dem_point(self, waypoints_wgs84, dem_path, sr_wgs84, messages):
        try:
            raster_sr = arcpy.Describe(dem_path).spatialReference
        except Exception:
            raster_sr = sr_wgs84
        elevs = []
        n = len(waypoints_wgs84)
        for i, (lon, lat) in enumerate(waypoints_wgs84):
            if i % 100 == 0:
                messages.addMessage("  Sampling: {}/{}".format(i, n))
            pt  = arcpy.PointGeometry(
                arcpy.Point(lon, lat), sr_wgs84).projectAs(raster_sr)
            res = arcpy.GetCellValue_management(
                dem_path, "{} {}".format(pt.firstPoint.X, pt.firstPoint.Y))
            val = res.getOutput(0)
            elevs.append(None if val in ("NoData", "") else float(val))
        return elevs

    # ==========================================================================
    # Map preview
    # ==========================================================================
    def _add_preview_to_map(self, all_waypoints, altitudes, meta, messages):
        try:
            arcpy.env.overwriteOutput = True
            scratch  = arcpy.env.scratchGDB
            sr_wgs84 = arcpy.SpatialReference(4326)

            pts_name = "DJI_Waypoints"
            pts_path = os.path.join(scratch, pts_name)
            if arcpy.Exists(pts_path):
                arcpy.management.Delete(pts_path)
            arcpy.management.CreateFeatureclass(
                scratch, pts_name, "POINT", spatial_reference=sr_wgs84)

            field_defs = [
                ("WP_INDEX",     "LONG",   None, "Waypoint sequence number"),
                ("STRIP_NUM",    "LONG",   None, "Lawnmower strip number"),
                ("GRID_TYPE",    "TEXT",   20,   "Primary or Cross-hatch"),
                ("ALTITUDE",     "DOUBLE", None, "Execute height above takeoff (m)"),
                ("TERRAIN_ELEV", "DOUBLE", None, "Terrain elevation at point (m ASL)"),
                ("TERRAIN_ADJ",  "DOUBLE", None, "Altitude adjustment for terrain (m)"),
                ("TAKE_PHOTO",   "TEXT",   3,    "Photo trigger"),
                ("HOVER_S",      "DOUBLE", None, "Hover seconds at this waypoint"),
                ("GIMBAL_PITCH", "DOUBLE", None, "Gimbal pitch angle (deg)"),
                ("SPEED",        "DOUBLE", None, "Waypoint speed (m/s)"),
                ("HEADING_MODE", "TEXT",   30,   "Drone heading mode"),
                ("DIST_PREV",    "DOUBLE", None, "Distance from previous waypoint (m)"),
                ("CUMUL_DIST",   "DOUBLE", None, "Cumulative flight distance (m)"),
                ("LON",          "DOUBLE", None, "Longitude WGS84"),
                ("LAT",          "DOUBLE", None, "Latitude WGS84"),
            ]
            for fname, ftype, flen, falias in field_defs:
                if flen:
                    arcpy.management.AddField(
                        pts_path, fname, ftype,
                        field_length=flen, field_alias=falias)
                else:
                    arcpy.management.AddField(
                        pts_path, fname, ftype, field_alias=falias)

            insert_fields = [f[0] for f in field_defs]
            insert_fields = ["SHAPE@XY"] + insert_fields

            strip_nums   = meta.get("strip_nums", [])
            grid_types   = meta.get("grid_types", [])
            raw_wp_elevs = meta.get("raw_wp_elevs")
            home_elev    = meta.get("home_elev")
            gimbal_pitch = meta["gimbal_pitch"]
            speed        = meta["speed"]
            heading_mode = meta["heading_mode"]
            hover_time   = meta.get("hover_time", 0.0)
            ae_lock      = meta.get("ae_lock", False)

            cumul = 0.0
            with arcpy.da.InsertCursor(pts_path, insert_fields) as cur:
                for idx, ((lon, lat), alt) in enumerate(
                        zip(all_waypoints, altitudes)):
                    if idx == 0:
                        dist_prev = 0.0
                    else:
                        prev = all_waypoints[idx - 1]
                        dist_prev = self._haversine(
                            prev[0], prev[1], lon, lat)
                    cumul += dist_prev

                    terrain_elev = None
                    terrain_adj  = None
                    if raw_wp_elevs is not None and idx < len(raw_wp_elevs):
                        te = raw_wp_elevs[idx]
                        if te is not None:
                            terrain_elev = round(te, 2)
                            if home_elev is not None:
                                terrain_adj = round(te - home_elev, 2)

                    strip_num = strip_nums[idx] if idx < len(strip_nums) else 0
                    grid_type = (grid_types[idx]
                                 if idx < len(grid_types) else "Primary")

                    # Hover time at this waypoint (incl AE lock at idx 0)
                    hov = hover_time
                    if idx == 0 and ae_lock:
                        hov = max(hov, 3.0)

                    cur.insertRow([
                        (lon, lat),
                        idx, strip_num, grid_type, round(alt, 2),
                        terrain_elev, terrain_adj, "Yes",
                        round(hov, 1),
                        gimbal_pitch, speed, heading_mode,
                        round(dist_prev, 1), round(cumul, 1),
                        lon, lat,
                    ])

            messages.addMessage(
                "Waypoints feature class: {} ({} records)".format(
                    pts_path, len(all_waypoints)))

            # ---- Flight path polyline ----
            path_name = "DJI_FlightPath"
            path_path = os.path.join(scratch, path_name)
            if arcpy.Exists(path_path):
                arcpy.management.Delete(path_path)
            arcpy.management.CreateFeatureclass(
                scratch, path_name, "POLYLINE", spatial_reference=sr_wgs84)

            path_fields = [
                ("MISSION",     "TEXT",   80,  "Mission Name"),
                ("TOTAL_WP",    "LONG",   None,"Total Waypoints"),
                ("DIST_M",      "DOUBLE", None,"Total Distance (m)"),
                ("EST_MIN",     "DOUBLE", None,"Est. Flight Time (min)"),
                ("AREA_HA",     "DOUBLE", None,"Survey Area (ha)"),
                ("AREA_AC",     "DOUBLE", None,"Survey Area (acres)"),
                ("SPEED",       "DOUBLE", None,"Speed (m/s)"),
                ("ALTITUDE",    "DOUBLE", None,"Base Altitude AGL (m)"),
                ("GIMBAL",      "DOUBLE", None,"Gimbal Pitch (deg)"),
                ("HOVER_S",     "DOUBLE", None,"Hover at Waypoints (s)"),
                ("AE_LOCK",     "TEXT",   3,   "AE Lock at First WP"),
                ("FINISH_ACT",  "TEXT",   30,  "Finish Action"),
                ("RC_LOST",     "TEXT",   30,  "RC Lost Action"),
                ("TERRAIN_SRC", "TEXT",   40,  "Terrain Source"),
                ("DOUBLE_GRID", "TEXT",   3,   "Double Grid"),
                ("GENERATED",   "TEXT",   30,  "Generated At"),
            ]
            for fname, ftype, flen, falias in path_fields:
                if flen:
                    arcpy.management.AddField(
                        path_path, fname, ftype,
                        field_length=flen, field_alias=falias)
                else:
                    arcpy.management.AddField(
                        path_path, fname, ftype, field_alias=falias)

            pts_arr  = arcpy.Array(
                [arcpy.Point(lon, lat) for lon, lat in all_waypoints])
            polyline = arcpy.Polyline(pts_arr, sr_wgs84)
            dist     = self._total_distance(all_waypoints)
            est_min  = (dist / speed
                        + len(all_waypoints) * hover_time
                        + (3.0 if ae_lock else 0.0)) / 60.0
            survey_area_m2 = meta.get("survey_area_m2", 0.0)

            insert_path_fields = ["SHAPE@"] + [f[0] for f in path_fields]
            with arcpy.da.InsertCursor(path_path, insert_path_fields) as cur:
                cur.insertRow([
                    polyline,
                    meta.get("mission_name") or "(unnamed)",
                    len(all_waypoints),
                    round(dist, 1),
                    round(est_min, 2),
                    round(survey_area_m2 * HA_PER_M2, 4),
                    round(survey_area_m2 * ACRE_PER_M2, 4),
                    speed, altitudes[0], gimbal_pitch,
                    hover_time,
                    "Yes" if ae_lock else "No",
                    meta.get("finish_action", ""),
                    meta.get("rc_lost_action", ""),
                    meta.get("terrain_source") or "None",
                    "Yes" if any(g == "Cross-hatch" for g in grid_types) else "No",
                    datetime.now().strftime("%Y-%m-%d %H:%M"),
                ])

            # ---- Add layers to map ----
            try:
                aprx       = arcpy.mp.ArcGISProject("CURRENT")
                active_map = aprx.activeMap
                if not active_map:
                    messages.addWarningMessage(
                        "No active map - preview saved to scratch GDB.")
                    return
                for lyr in active_map.listLayers():
                    if lyr.name in (pts_name, path_name):
                        active_map.removeLayer(lyr)
                path_lyr = active_map.addDataFromPath(path_path)
                pts_lyr  = active_map.addDataFromPath(pts_path)

                try:
                    sym = pts_lyr.symbology
                    sym.updateRenderer('GraduatedColorsRenderer')
                    sym.renderer.classificationField = "ALTITUDE"
                    sym.renderer.breakCount = 5
                    for ramp_name in ['Yellow to Red', 'Blue to Red',
                                      'Cyan to Purple']:
                        candidates = aprx.listColorRamps(ramp_name)
                        if candidates:
                            sym.renderer.colorRamp = candidates[0]
                            break
                    pts_lyr.symbology = sym
                except Exception:
                    pass

                try:
                    sym2 = path_lyr.symbology
                    sym2.renderer.symbol.color = {'RGB': [46, 168, 213, 100]}
                    sym2.renderer.symbol.width = 1.0
                    path_lyr.symbology = sym2
                except Exception:
                    pass

                messages.addMessage(
                    "Preview layers added: {} and {}".format(pts_name, path_name))
            except RuntimeError as e:
                messages.addWarningMessage(
                    "arcpy.mp unavailable: {}. Layers in scratch GDB only.".format(e))
        except Exception as e:
            messages.addWarningMessage(
                "Preview failed: {}. KMZ is unaffected.".format(e))

    # ==========================================================================
    # Helpers
    # ==========================================================================
    @staticmethod
    def _haversine(lon1, lat1, lon2, lat2):
        R = 6_371_000.0
        lo1 = math.radians(lon1); la1 = math.radians(lat1)
        lo2 = math.radians(lon2); la2 = math.radians(lat2)
        a   = (math.sin((la2-la1)/2)**2
               + math.cos(la1)*math.cos(la2)*math.sin((lo2-lo1)/2)**2)
        return R * 2 * math.asin(math.sqrt(max(0.0, a)))

    @staticmethod
    def _total_distance(wps):
        R, total = 6_371_000.0, 0.0
        for i in range(1, len(wps)):
            lo1, la1 = math.radians(wps[i-1][0]), math.radians(wps[i-1][1])
            lo2, la2 = math.radians(wps[i][0]),   math.radians(wps[i][1])
            a = (math.sin((la2-la1)/2)**2
                 + math.cos(la1)*math.cos(la2)*math.sin((lo2-lo1)/2)**2)
            total += R * 2 * math.asin(math.sqrt(max(0.0, a)))
        return total

    # ==========================================================================
    # KMZ / WPML writers
    # ==========================================================================
    def _write_kmz(self, waypoints, altitudes, output_kmz, speed,
                   gimbal_pitch, heading_mode, finish_action, rc_lost_action,
                   hover_time, ae_lock, mission_name):
        ts = int(datetime.now().timestamp() * 1000)
        with zipfile.ZipFile(output_kmz, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("wpmz/waylines.wpml",
                self._build_wpml(
                    waypoints, altitudes, speed, gimbal_pitch,
                    heading_mode, finish_action, rc_lost_action,
                    hover_time, ae_lock, mission_name, ts))
            zf.writestr("wpmz/template.kml",
                self._build_template_kml(
                    waypoints, altitudes, speed, gimbal_pitch,
                    mission_name, ts))

    def _build_wpml(self, waypoints, altitudes, speed, gimbal_pitch,
                    heading_mode, finish_action, rc_lost_action,
                    hover_time, ae_lock, mission_name, ts):
        L = []; a = L.append
        author_str = (mission_name + " (DJI Mission Generator v5)"
                      if mission_name else "ArcGIS Pro DJI Mission Generator v5")

        a('<?xml version="1.0" encoding="UTF-8"?>')
        a('<kml xmlns="http://www.opengis.net/kml/2.2"')
        a('     xmlns:wpml="http://www.dji.com/wpmz/1.0.2">')
        a('  <Document>')
        a('    <wpml:author>{}</wpml:author>'.format(self._xml_escape(author_str)))
        a('    <wpml:createTime>{}</wpml:createTime>'.format(ts))
        a('    <wpml:updateTime>{}</wpml:updateTime>'.format(ts))
        a('    <wpml:missionConfig>')
        a('      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>')
        a('      <wpml:finishAction>{}</wpml:finishAction>'.format(finish_action))
        a('      <wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost>')
        a('      <wpml:executeRCLostAction>{}</wpml:executeRCLostAction>'.format(rc_lost_action))
        a('      <wpml:globalTransitionalSpeed>{:.1f}</wpml:globalTransitionalSpeed>'.format(speed))
        a('      <wpml:droneInfo>')
        a('        <wpml:droneEnumValue>67</wpml:droneEnumValue>')
        a('        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>')
        a('      </wpml:droneInfo>')
        a('    </wpml:missionConfig>')
        a('    <Folder>')
        a('      <wpml:templateId>0</wpml:templateId>')
        a('      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>')
        a('      <wpml:waylineId>0</wpml:waylineId>')
        a('      <wpml:distance>0</wpml:distance>')
        a('      <wpml:duration>0</wpml:duration>')
        a('      <wpml:autoFlightSpeed>{:.1f}</wpml:autoFlightSpeed>'.format(speed))

        for idx, ((lon, lat), alt) in enumerate(zip(waypoints, altitudes)):
            # Determine hover for this waypoint
            wp_hover = hover_time
            if idx == 0 and ae_lock:
                wp_hover = max(wp_hover, 3.0)

            a('      <Placemark>')
            a('        <Point><coordinates>{:.8f},{:.8f}</coordinates></Point>'.format(lon, lat))
            a('        <wpml:index>{}</wpml:index>'.format(idx))
            a('        <wpml:executeHeight>{:.2f}</wpml:executeHeight>'.format(alt))
            a('        <wpml:waypointSpeed>{:.1f}</wpml:waypointSpeed>'.format(speed))
            a('        <wpml:waypointHeadingParam>')
            a('          <wpml:waypointHeadingMode>{}</wpml:waypointHeadingMode>'.format(heading_mode))
            a('          <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>')
            a('          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>')
            a('          <wpml:waypointHeadingAngleEnable>0</wpml:waypointHeadingAngleEnable>')
            a('          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>')
            a('        </wpml:waypointHeadingParam>')
            a('        <wpml:waypointTurnParam>')
            a('          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>')
            a('          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>')
            a('        </wpml:waypointTurnParam>')
            a('        <wpml:useStraightLine>1</wpml:useStraightLine>')

            # Action group
            a('        <wpml:actionGroup>')
            a('          <wpml:actionGroupId>{}</wpml:actionGroupId>'.format(idx))
            a('          <wpml:actionGroupStartIndex>{}</wpml:actionGroupStartIndex>'.format(idx))
            a('          <wpml:actionGroupEndIndex>{}</wpml:actionGroupEndIndex>'.format(idx))
            a('          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>')
            a('          <wpml:actionTrigger>')
            a('            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>')
            a('          </wpml:actionTrigger>')

            action_id = 0

            # Action: gimbal rotate
            a('          <wpml:action>')
            a('            <wpml:actionId>{}</wpml:actionId>'.format(action_id))
            a('            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>')
            a('            <wpml:actionActuatorFuncParam>')
            a('              <wpml:gimbalHeadingYawBase>aircraft</wpml:gimbalHeadingYawBase>')
            a('              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>')
            a('              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>')
            a('              <wpml:gimbalPitchRotateAngle>{:.1f}</wpml:gimbalPitchRotateAngle>'.format(gimbal_pitch))
            a('              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>')
            a('              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>')
            a('              <wpml:gimbalYawRotateEnable>0</wpml:gimbalYawRotateEnable>')
            a('              <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>')
            a('              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>')
            a('              <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>')
            a('              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>')
            a('            </wpml:actionActuatorFuncParam>')
            a('          </wpml:action>')
            action_id += 1

            # Action: hover (if hover time > 0)
            if wp_hover > 0:
                a('          <wpml:action>')
                a('            <wpml:actionId>{}</wpml:actionId>'.format(action_id))
                a('            <wpml:actionActuatorFunc>hover</wpml:actionActuatorFunc>')
                a('            <wpml:actionActuatorFuncParam>')
                a('              <wpml:hoverTime>{:.1f}</wpml:hoverTime>'.format(wp_hover))
                a('            </wpml:actionActuatorFuncParam>')
                a('          </wpml:action>')
                action_id += 1

            # Action: take photo
            a('          <wpml:action>')
            a('            <wpml:actionId>{}</wpml:actionId>'.format(action_id))
            a('            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>')
            a('            <wpml:actionActuatorFuncParam>')
            a('              <wpml:fileSuffix>waypoint</wpml:fileSuffix>')
            a('              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>')
            a('            </wpml:actionActuatorFuncParam>')
            a('          </wpml:action>')

            a('        </wpml:actionGroup>')
            a('      </Placemark>')

        a('    </Folder>')
        a('  </Document>')
        a('</kml>')
        return "\n".join(L)

    def _build_template_kml(self, waypoints, altitudes, speed, gimbal_pitch,
                            mission_name, ts):
        L = []; a = L.append
        mean_alt = sum(altitudes) / len(altitudes)
        author_str = (mission_name + " (DJI Mission Generator v5)"
                      if mission_name else "ArcGIS Pro DJI Mission Generator v5")
        a('<?xml version="1.0" encoding="UTF-8"?>')
        a('<kml xmlns="http://www.opengis.net/kml/2.2"')
        a('     xmlns:wpml="http://www.dji.com/wpmz/1.0.2">')
        a('  <Document>')
        a('    <wpml:author>{}</wpml:author>'.format(self._xml_escape(author_str)))
        a('    <wpml:createTime>{}</wpml:createTime>'.format(ts))
        a('    <wpml:updateTime>{}</wpml:updateTime>'.format(ts))
        a('    <wpml:missionConfig>')
        a('      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>')
        a('      <wpml:finishAction>goHome</wpml:finishAction>')
        a('      <wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost>')
        a('      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>')
        a('      <wpml:globalTransitionalSpeed>{:.1f}</wpml:globalTransitionalSpeed>'.format(speed))
        a('      <wpml:droneInfo>')
        a('        <wpml:droneEnumValue>67</wpml:droneEnumValue>')
        a('        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>')
        a('      </wpml:droneInfo>')
        a('    </wpml:missionConfig>')
        a('    <Folder>')
        a('      <wpml:templateId>0</wpml:templateId>')
        a('      <wpml:templateType>waypoint</wpml:templateType>')
        a('      <wpml:waylineCoordinateSysParam>')
        a('        <wpml:coordinateMode>WGS84</wpml:coordinateMode>')
        a('        <wpml:heightMode>relativeToStartPoint</wpml:heightMode>')
        a('      </wpml:waylineCoordinateSysParam>')
        a('      <wpml:autoFlightSpeed>{:.1f}</wpml:autoFlightSpeed>'.format(speed))
        a('      <wpml:globalHeight>{:.1f}</wpml:globalHeight>'.format(mean_alt))
        a('      <wpml:caliFlightEnable>0</wpml:caliFlightEnable>')
        a('      <wpml:gimbalPitchMode>usePointSetting</wpml:gimbalPitchMode>')

        for idx, ((lon, lat), alt) in enumerate(zip(waypoints, altitudes)):
            a('      <Placemark>')
            a('        <Point><coordinates>{:.8f},{:.8f}</coordinates></Point>'.format(lon, lat))
            a('        <wpml:index>{}</wpml:index>'.format(idx))
            a('        <wpml:ellipsoidHeight>{:.2f}</wpml:ellipsoidHeight>'.format(alt))
            a('        <wpml:height>{:.2f}</wpml:height>'.format(alt))
            a('        <wpml:useGlobalHeight>0</wpml:useGlobalHeight>')
            a('        <wpml:useGlobalSpeed>1</wpml:useGlobalSpeed>')
            a('        <wpml:useGlobalHeadingParam>1</wpml:useGlobalHeadingParam>')
            a('        <wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam>')
            a('        <wpml:gimbalPitchAngle>{:.1f}</wpml:gimbalPitchAngle>'.format(gimbal_pitch))
            a('      </Placemark>')

        a('    </Folder>')
        a('  </Document>')
        a('</kml>')
        return "\n".join(L)

    @staticmethod
    def _xml_escape(s):
        return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
                 .replace('"', "&quot;").replace("'", "&apos;"))
