# -*- coding: utf-8 -*-
"""
DJI Mini 4 Pro Waypoint Mission Generator
ArcGIS Pro Python Toolbox (.pyt)

Generates a DJI WPML (.kmz) waypoint mission from a polygon shapefile.
The drone flies a boustrophedon (lawnmower) grid, taking a photo at a
user-defined distance interval along each strip.

Output: DJI WPML 1.0.2 .kmz  — load via DJI Pilot 2 -> Mission -> Waypoint -> Import
Compatible: DJI Mini 4 Pro (droneEnumValue=67) + RC Pro controller
"""

import arcpy
import math
import zipfile
import os
from datetime import datetime


class Toolbox(object):
    def __init__(self):
        self.label = "DJI Waypoint Mission Generator"
        self.alias = "DJIWaypointMission"
        self.tools = [GenerateWaypointMission]


class GenerateWaypointMission(object):

    def __init__(self):
        self.label = "Generate DJI Waypoint Mission (Mini 4 Pro)"
        self.description = (
            "Creates a DJI WPML (.kmz) waypoint mission from a polygon. "
            "The drone flies a lawnmower grid and takes a photo at a "
            "specified distance interval along each pass."
        )
        self.canRunInBackground = False

    def getParameterInfo(self):
        params = []

        # 0 - Input polygon
        p = arcpy.Parameter(
            displayName="Input Polygon Feature",
            name="input_feature",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input"
        )
        params.append(p)

        # 1 - Output KMZ
        p = arcpy.Parameter(
            displayName="Output KMZ File",
            name="output_kmz",
            datatype="DEFile",
            parameterType="Required",
            direction="Output"
        )
        p.filter.list = ["kmz"]
        params.append(p)

        # 2 - Altitude
        p = arcpy.Parameter(
            displayName="Flight Altitude AGL (metres)",
            name="altitude",
            datatype="GPDouble",
            parameterType="Required",
            direction="Input"
        )
        p.value = 60.0
        params.append(p)

        # 3 - Speed
        p = arcpy.Parameter(
            displayName="Flight Speed (m/s,  max 15)",
            name="speed",
            datatype="GPDouble",
            parameterType="Required",
            direction="Input"
        )
        p.value = 8.0
        params.append(p)

        # 4 - Strip spacing
        p = arcpy.Parameter(
            displayName="Strip Spacing - distance between parallel passes (metres)",
            name="strip_spacing",
            datatype="GPDouble",
            parameterType="Required",
            direction="Input"
        )
        p.value = 30.0
        params.append(p)

        # 5 - Photo interval
        p = arcpy.Parameter(
            displayName="Photo Interval - distance between photos along each pass (metres)",
            name="photo_interval",
            datatype="GPDouble",
            parameterType="Required",
            direction="Input"
        )
        p.value = 20.0
        params.append(p)

        # 6 - Strip direction
        p = arcpy.Parameter(
            displayName="Strip Direction - bearing of each pass, degrees from North (0-360)",
            name="flight_direction",
            datatype="GPDouble",
            parameterType="Required",
            direction="Input"
        )
        p.value = 0.0
        params.append(p)

        # 7 - Gimbal pitch
        p = arcpy.Parameter(
            displayName="Gimbal Pitch Angle (degrees)   -90 = straight down   0 = forward",
            name="gimbal_pitch",
            datatype="GPDouble",
            parameterType="Required",
            direction="Input"
        )
        p.value = -90.0
        params.append(p)

        # 8 - Heading mode
        p = arcpy.Parameter(
            displayName="Drone Heading Mode",
            name="heading_mode",
            datatype="GPString",
            parameterType="Required",
            direction="Input"
        )
        p.filter.type = "ValueList"
        p.filter.list = ["followWayline", "fixed", "smoothTransition"]
        p.value = "followWayline"
        params.append(p)

        # 9 - Finish action
        p = arcpy.Parameter(
            displayName="Mission Finish Action",
            name="finish_action",
            datatype="GPString",
            parameterType="Required",
            direction="Input"
        )
        p.filter.type = "ValueList"
        p.filter.list = ["goHome", "autoLand", "gotoFirstWaypoint"]
        p.value = "goHome"
        params.append(p)

        # 10 - RC-lost action
        p = arcpy.Parameter(
            displayName="RC Signal Lost Action",
            name="rc_lost_action",
            datatype="GPString",
            parameterType="Required",
            direction="Input"
        )
        p.filter.type = "ValueList"
        p.filter.list = ["goBack", "hover", "land", "goContinue"]
        p.value = "goBack"
        params.append(p)

        return params

    def isLicensed(self):
        return True

    def updateParameters(self, parameters):
        return

    def updateMessages(self, parameters):
        altitude       = parameters[2].value
        speed          = parameters[3].value
        strip_spacing  = parameters[4].value
        photo_interval = parameters[5].value
        gimbal_pitch   = parameters[7].value

        if altitude is not None:
            if altitude < 2:
                parameters[2].setWarningMessage("Altitude below 2 m - confirm this is intentional.")
            elif altitude > 120:
                parameters[2].setWarningMessage(
                    "Altitude exceeds 120 m AGL. Ensure you hold a Transport Canada "
                    "SFOC or Advanced Operations certificate."
                )

        if speed is not None and speed > 15:
            parameters[3].setErrorMessage("DJI Mini 4 Pro maximum waypoint speed is 15 m/s.")

        if strip_spacing is not None and strip_spacing < 1:
            parameters[4].setErrorMessage("Strip spacing must be at least 1 m.")

        if photo_interval is not None and photo_interval < 1:
            parameters[5].setErrorMessage("Photo interval must be at least 1 m.")

        if gimbal_pitch is not None and not (-90 <= gimbal_pitch <= 30):
            parameters[7].setWarningMessage(
                "Unusual gimbal pitch. Typical range: -90 (nadir) to 0 (forward)."
            )

    def execute(self, parameters, messages):
        input_feature  = parameters[0].valueAsText
        output_kmz     = parameters[1].valueAsText
        altitude       = float(parameters[2].value)
        speed          = float(parameters[3].value)
        strip_spacing  = float(parameters[4].value)
        photo_interval = float(parameters[5].value)
        flight_dir_deg = float(parameters[6].value)
        gimbal_pitch   = float(parameters[7].value)
        heading_mode   = parameters[8].valueAsText
        finish_action  = parameters[9].valueAsText
        rc_lost_action = parameters[10].valueAsText

        if not output_kmz.lower().endswith(".kmz"):
            output_kmz += ".kmz"

        sr_albers = arcpy.SpatialReference(3005)   # BC Albers - metric
        sr_wgs84  = arcpy.SpatialReference(4326)

        desc = arcpy.Describe(input_feature)
        if desc.shapeType != "Polygon":
            messages.addErrorMessage(
                "Input feature must be a Polygon layer. Got: {}".format(desc.shapeType)
            )
            return

        all_waypoints = []

        with arcpy.da.SearchCursor(input_feature, ["SHAPE@", "OID@"]) as cursor:
            for row in cursor:
                geom, oid = row
                geom_albers = geom.projectAs(sr_albers)
                messages.addMessage("Processing feature OID {}...".format(oid))
                wpts = self._lawnmower(
                    geom_albers, strip_spacing, photo_interval,
                    flight_dir_deg, sr_albers, sr_wgs84, messages
                )
                all_waypoints.extend(wpts)

        if not all_waypoints:
            messages.addErrorMessage(
                "No waypoints were generated. Check that the polygon has valid geometry "
                "and that strip spacing / photo interval are not larger than the polygon."
            )
            return

        n = len(all_waypoints)
        messages.addMessage("Total waypoints: {}".format(n))

        if n > 65535:
            messages.addErrorMessage(
                "{} waypoints exceeds the DJI WPML maximum of 65,535. "
                "Increase strip spacing or photo interval, or split the survey area.".format(n)
            )
            return

        if n > 2000:
            messages.addWarningMessage(
                "{} waypoints is a large mission. "
                "Consider splitting the area into smaller blocks.".format(n)
            )

        est_dist = self._total_distance(all_waypoints)
        est_time = est_dist / speed / 60.0
        messages.addMessage(
            "Estimated flight path: {:.0f} m  (~{:.1f} min at {} m/s)".format(
                est_dist, est_time, speed
            )
        )

        out_dir = os.path.dirname(output_kmz)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir)

        self._write_kmz(
            all_waypoints, output_kmz, altitude, speed,
            gimbal_pitch, heading_mode, finish_action, rc_lost_action
        )

        messages.addMessage("\nKMZ written: {}".format(output_kmz))
        messages.addMessage(
            "Load on RC Pro: DJI Pilot 2 -> Mission -> Waypoint -> Import -> select the .kmz"
        )

    # -------------------------------------------------------------------------
    # Lawnmower grid
    # -------------------------------------------------------------------------

    @staticmethod
    def _rotate(x, y, cx, cy, angle_rad):
        """Rotate (x,y) around (cx,cy) by angle_rad (CCW positive)."""
        dx, dy = x - cx, y - cy
        rx = dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
        ry = dx * math.sin(angle_rad) + dy * math.cos(angle_rad)
        return rx + cx, ry + cy

    def _lawnmower(self, polygon_albers, strip_spacing, photo_interval,
                   flight_dir_deg, sr_albers, sr_wgs84, messages):
        """
        Boustrophedon (back-and-forth) grid inside a polygon.

        The polygon is rotated so the desired flight bearing aligns with
        the X axis. Strips run parallel to X, spaced strip_spacing apart.
        Photo waypoints are placed every photo_interval metres along each
        strip. Strips alternate direction. The grid is then rotated back
        and projected to WGS84.
        """
        extent = polygon_albers.extent
        cx = (extent.XMin + extent.XMax) / 2.0
        cy = (extent.YMin + extent.YMax) / 2.0

        # Convert compass bearing to CCW-from-East math angle
        math_angle   =  math.radians(90.0 - flight_dir_deg)
        rot_to_grid  =  math_angle
        rot_to_world = -math_angle

        # Exterior ring vertices
        part  = polygon_albers.getPart(0)
        verts = [(pnt.X, pnt.Y) for pnt in part if pnt is not None]
        if len(verts) < 3:
            messages.addWarningMessage("Polygon has fewer than 3 vertices - skipped.")
            return []
        if verts[0] != verts[-1]:
            verts.append(verts[0])

        # Rotate vertices into grid frame
        rv = [self._rotate(x, y, cx, cy, rot_to_grid) for x, y in verts]

        min_y = min(v[1] for v in rv)
        max_y = max(v[1] for v in rv)

        edges = list(zip(rv[:-1], rv[1:]))

        # Strip Y positions (centre of first strip = min_y + half spacing)
        strip_ys = []
        y = min_y + strip_spacing / 2.0
        while y <= max_y + 1e-6:
            strip_ys.append(y)
            y += strip_spacing

        if not strip_ys:
            messages.addWarningMessage(
                "No strips fit inside this polygon with the current strip spacing."
            )
            return []

        waypoints = []

        for strip_idx, sy in enumerate(strip_ys):

            # --- Scanline intersection at Y = sy ---
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

            # Process intersection pairs (handles concave polygons)
            for pi in range(0, len(xs) - 1, 2):
                x_a = xs[pi]
                x_b = xs[pi + 1]
                seg  = abs(x_b - x_a)
                if seg < 0.1:
                    continue

                # Alternate direction each strip
                if strip_idx % 2 == 1:
                    x_a, x_b = x_b, x_a

                direction = 1 if x_b >= x_a else -1
                seg       = abs(x_b - x_a)

                # Number of intervals — always land on both ends of the strip
                n_intervals = max(1, int(math.ceil(seg / photo_interval)))
                actual_ivl  = seg / n_intervals

                for k in range(n_intervals + 1):
                    gx = x_a + direction * k * actual_ivl
                    wx, wy = self._rotate(gx, sy, cx, cy, rot_to_world)
                    pt    = arcpy.PointGeometry(arcpy.Point(wx, wy), sr_albers)
                    pt_wg = pt.projectAs(sr_wgs84)
                    waypoints.append((pt_wg.firstPoint.X, pt_wg.firstPoint.Y))

        messages.addMessage(
            "  -> {} strips, {} waypoints".format(len(strip_ys), len(waypoints))
        )
        return waypoints

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def _total_distance(waypoints):
        R = 6_371_000.0
        total = 0.0
        for i in range(1, len(waypoints)):
            lo1 = math.radians(waypoints[i-1][0]);  la1 = math.radians(waypoints[i-1][1])
            lo2 = math.radians(waypoints[i][0]);    la2 = math.radians(waypoints[i][1])
            a = (math.sin((la2-la1)/2)**2
                 + math.cos(la1) * math.cos(la2) * math.sin((lo2-lo1)/2)**2)
            total += R * 2 * math.asin(math.sqrt(max(0.0, a)))
        return total

    # -------------------------------------------------------------------------
    # KMZ / WPML
    # -------------------------------------------------------------------------

    def _write_kmz(self, waypoints, output_kmz, altitude, speed,
                   gimbal_pitch, heading_mode, finish_action, rc_lost_action):
        ts = int(datetime.now().timestamp() * 1000)
        with zipfile.ZipFile(output_kmz, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("wpmz/waylines.wpml",
                        self._build_wpml(waypoints, altitude, speed, gimbal_pitch,
                                         heading_mode, finish_action, rc_lost_action, ts))
            zf.writestr("wpmz/template.kml",
                        self._build_template_kml(waypoints, altitude, speed, gimbal_pitch, ts))

    def _build_wpml(self, waypoints, altitude, speed, gimbal_pitch,
                    heading_mode, finish_action, rc_lost_action, ts):
        L = []
        a = L.append
        a('<?xml version="1.0" encoding="UTF-8"?>')
        a('<kml xmlns="http://www.opengis.net/kml/2.2"')
        a('     xmlns:wpml="http://www.dji.com/wpmz/1.0.2">')
        a('  <Document>')
        a('    <wpml:author>ArcGIS Pro DJI Mission Generator</wpml:author>')
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

        for idx, (lon, lat) in enumerate(waypoints):
            a('      <Placemark>')
            a('        <Point><coordinates>{:.8f},{:.8f}</coordinates></Point>'.format(lon, lat))
            a('        <wpml:index>{}</wpml:index>'.format(idx))
            a('        <wpml:executeHeight>{:.1f}</wpml:executeHeight>'.format(altitude))
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
            a('        <wpml:actionGroup>')
            a('          <wpml:actionGroupId>{}</wpml:actionGroupId>'.format(idx))
            a('          <wpml:actionGroupStartIndex>{}</wpml:actionGroupStartIndex>'.format(idx))
            a('          <wpml:actionGroupEndIndex>{}</wpml:actionGroupEndIndex>'.format(idx))
            a('          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>')
            a('          <wpml:actionTrigger>')
            a('            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>')
            a('          </wpml:actionTrigger>')
            # Action 0: gimbal pitch
            a('          <wpml:action>')
            a('            <wpml:actionId>0</wpml:actionId>')
            a('            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>')
            a('            <wpml:actionActuatorFuncParam>')
            a('              <wpml:gimbalPitchRotateAngle>{:.1f}</wpml:gimbalPitchRotateAngle>'.format(gimbal_pitch))
            a('              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>')
            a('              <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>')
            a('              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>')
            a('              <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>')
            a('              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>')
            a('            </wpml:actionActuatorFuncParam>')
            a('          </wpml:action>')
            # Action 1: take photo
            a('          <wpml:action>')
            a('            <wpml:actionId>1</wpml:actionId>')
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

    def _build_template_kml(self, waypoints, altitude, speed, gimbal_pitch, ts):
        L = []
        a = L.append
        a('<?xml version="1.0" encoding="UTF-8"?>')
        a('<kml xmlns="http://www.opengis.net/kml/2.2"')
        a('     xmlns:wpml="http://www.dji.com/wpmz/1.0.2">')
        a('  <Document>')
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
        a('      <wpml:globalHeight>{:.1f}</wpml:globalHeight>'.format(altitude))
        a('      <wpml:caliFlightEnable>0</wpml:caliFlightEnable>')
        a('      <wpml:gimbalPitchMode>usePointSetting</wpml:gimbalPitchMode>')

        for idx, (lon, lat) in enumerate(waypoints):
            a('      <Placemark>')
            a('        <Point><coordinates>{:.8f},{:.8f}</coordinates></Point>'.format(lon, lat))
            a('        <wpml:index>{}</wpml:index>'.format(idx))
            a('        <wpml:ellipsoidHeight>{:.1f}</wpml:ellipsoidHeight>'.format(altitude))
            a('        <wpml:height>{:.1f}</wpml:height>'.format(altitude))
            a('        <wpml:useGlobalHeight>1</wpml:useGlobalHeight>')
            a('        <wpml:useGlobalSpeed>1</wpml:useGlobalSpeed>')
            a('        <wpml:useGlobalHeadingParam>1</wpml:useGlobalHeadingParam>')
            a('        <wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam>')
            a('        <wpml:gimbalPitchAngle>{:.1f}</wpml:gimbalPitchAngle>'.format(gimbal_pitch))
            a('      </Placemark>')

        a('    </Folder>')
        a('  </Document>')
        a('</kml>')
        return "\n".join(L)
