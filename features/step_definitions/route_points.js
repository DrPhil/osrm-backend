// Step definitions for testing route-points feature in trip planning API
import { When, Then } from '@cucumber/cucumber';
import polyline from '@mapbox/polyline';

// Helper to decode geometry based on format
function decodeGeometry(geometry, format) {
  if (format === 'polyline') {
    return polyline.decode(geometry);
  } else if (format === 'polyline6') {
    return polyline.decode(geometry, 6);
  } else {
    // GeoJSON format - coordinates are already an array
    return geometry.coordinates;
  }
}

// Helper to compare two geometries with tolerance
function geometriesMatch(geom1, geom2, tolerance = 0.0001) {
  if (geom1.length !== geom2.length) {
    return false;
  }
  for (let i = 0; i < geom1.length; i++) {
    const [lat1, lon1] = Array.isArray(geom1[i][0]) ? [geom1[i][1], geom1[i][0]] : geom1[i];
    const [lat2, lon2] = Array.isArray(geom2[i][0]) ? [geom2[i][1], geom2[i][0]] : geom2[i];
    if (Math.abs(lat1 - lat2) > tolerance || Math.abs(lon1 - lon2) > tolerance) {
      return false;
    }
  }
  return true;
}

// Helper to normalize geometry format (handle both polyline [lat,lon] and geojson [lon,lat])
function normalizeGeometry(coords, isGeoJSON) {
  if (isGeoJSON) {
    // GeoJSON uses [lon, lat], convert to [lat, lon] for comparison
    return coords.map(c => [c[1], c[0]]);
  }
  return coords;
}

When(/^I plan a trip with route-points I should get a sufficient set$/, function (table, callback) {
  this.reprocessAndLoadData((e) => {
    if (e) return callback(e);

    const testRow = (row, ri, cb) => {
      const waypoints = [];
      row.waypoints.split(',').forEach((n) => {
        const node = this.findNodeByName(n.trim());
        if (!node) throw new Error(`*** unknown waypoint node "${n.trim()}"`);
        waypoints.push(node);
      });

      const params = Object.assign({}, this.queryParams);
      params['route-points'] = 'true';
      params.steps = 'true';
      params.geometries = params.geometries || 'geojson';
      params.overview = 'full';

      if (row.source) params.source = row.source;
      if (row.destination) params.destination = row.destination;
      if (row.hasOwnProperty('roundtrip')) params.roundtrip = row.roundtrip;

      // Step 1: Make trip request with route-points=true
      this.requestTrip(waypoints, params, (err, tripRes, tripBody) => {
        if (err) return cb(err);

        let tripJson;
        try {
          tripJson = JSON.parse(tripBody);
        } catch (parseErr) {
          return cb(new Error(`Failed to parse trip response: ${parseErr.message}`));
        }

        if (tripJson.code !== 'Ok') {
          return cb(null, { waypoints: row.waypoints, status: tripJson.code, sufficient: 'N/A' });
        }

        if (!tripJson.route_points || !tripJson.route_points[0]) {
          return cb(new Error('No route_points in trip response'));
        }

        const tripGeometry = tripJson.trips[0].geometry;
        const routePoints = tripJson.route_points[0];

        // Step 2: Use route_points to make a route request
        const routeWaypoints = routePoints.map(rp => ({
          lon: rp.location[0],
          lat: rp.location[1]
        }));

        const routeParams = {
          output: 'json',
          steps: 'true',
          geometries: params.geometries,
          overview: 'full'
        };

        this.requestRoute(routeWaypoints, [], [], routeParams, (routeErr, routeRes, routeBody) => {
          if (routeErr) return cb(routeErr);

          let routeJson;
          try {
            routeJson = JSON.parse(routeBody);
          } catch (parseErr) {
            return cb(new Error(`Failed to parse route response: ${parseErr.message}`));
          }

          if (routeJson.code !== 'Ok') {
            return cb(null, {
              waypoints: row.waypoints,
              status: 'Ok',
              sufficient: 'no (route failed)',
              route_points_count: routePoints.length
            });
          }

          // Step 3: Compare geometries
          const routeGeometry = routeJson.routes[0].geometry;
          const isGeoJSON = params.geometries === 'geojson';

          let tripCoords, routeCoords;
          if (isGeoJSON) {
            tripCoords = tripGeometry.coordinates;
            routeCoords = routeGeometry.coordinates;
          } else {
            const precision = params.geometries === 'polyline6' ? 6 : 5;
            tripCoords = polyline.decode(tripGeometry, precision);
            routeCoords = polyline.decode(routeGeometry, precision);
          }

          // Normalize to same format for comparison
          const normalizedTrip = normalizeGeometry(tripCoords, isGeoJSON);
          const normalizedRoute = normalizeGeometry(routeCoords, isGeoJSON);

          const sufficient = geometriesMatch(normalizedTrip, normalizedRoute);

          const got = {
            waypoints: row.waypoints,
            status: 'Ok',
            sufficient: sufficient ? 'yes' : 'no',
            route_points_count: routePoints.length
          };

          // Match against expected values
          for (const key in row) {
            if (this.FuzzyMatch.match(String(got[key]), row[key])) {
              got[key] = row[key];
            }
          }

          cb(null, got);
        });
      });
    };

    this.processRowsAndDiff(table, testRow, callback);
  });
});

When(/^I plan a trip with route-points I should get a locally minimal set$/, function (table, callback) {
  this.reprocessAndLoadData((e) => {
    if (e) return callback(e);

    const testRow = (row, ri, cb) => {
      const waypoints = [];
      row.waypoints.split(',').forEach((n) => {
        const node = this.findNodeByName(n.trim());
        if (!node) throw new Error(`*** unknown waypoint node "${n.trim()}"`);
        waypoints.push(node);
      });

      const params = Object.assign({}, this.queryParams);
      params['route-points'] = 'true';
      params.steps = 'true';
      params.geometries = params.geometries || 'geojson';
      params.overview = 'full';

      if (row.source) params.source = row.source;
      if (row.destination) params.destination = row.destination;
      if (row.hasOwnProperty('roundtrip')) params.roundtrip = row.roundtrip;

      // Step 1: Make trip request with route-points=true
      this.requestTrip(waypoints, params, (err, tripRes, tripBody) => {
        if (err) return cb(err);

        let tripJson;
        try {
          tripJson = JSON.parse(tripBody);
        } catch (parseErr) {
          return cb(new Error(`Failed to parse trip response: ${parseErr.message}`));
        }

        if (tripJson.code !== 'Ok') {
          return cb(null, { waypoints: row.waypoints, status: tripJson.code, minimal: 'N/A' });
        }

        if (!tripJson.route_points || !tripJson.route_points[0]) {
          return cb(new Error('No route_points in trip response'));
        }

        const tripGeometry = tripJson.trips[0].geometry;
        const routePoints = tripJson.route_points[0];
        const isGeoJSON = params.geometries === 'geojson';

        // Get the original trip geometry for comparison
        let originalTripCoords;
        if (isGeoJSON) {
          originalTripCoords = tripGeometry.coordinates;
        } else {
          const precision = params.geometries === 'polyline6' ? 6 : 5;
          originalTripCoords = polyline.decode(tripGeometry, precision);
        }
        const normalizedOriginal = normalizeGeometry(originalTripCoords, isGeoJSON);

        // Step 2: For each interior point (not first or last), try removing it
        // and verify the route changes
        // Note: We only test interior points because removing endpoints would definitely change the route
        const interiorPoints = routePoints.slice(1, -1);

        if (interiorPoints.length === 0) {
          // Only 2 points (start and end), trivially minimal
          return cb(null, {
            waypoints: row.waypoints,
            status: 'Ok',
            minimal: 'yes',
            route_points_count: routePoints.length,
            tested_removals: 0
          });
        }

        let allRemovalsChangeRoute = true;
        let testedCount = 0;
        let failedRemovalIndex = -1;

        const testRemoval = (index) => {
          if (index >= interiorPoints.length) {
            // All removals tested
            const got = {
              waypoints: row.waypoints,
              status: 'Ok',
              minimal: allRemovalsChangeRoute ? 'yes' : 'no',
              route_points_count: routePoints.length,
              tested_removals: testedCount
            };

            if (!allRemovalsChangeRoute) {
              got.failed_at_index = failedRemovalIndex + 1; // +1 because we skip first point
            }

            for (const key in row) {
              if (this.FuzzyMatch.match(String(got[key]), row[key])) {
                got[key] = row[key];
              }
            }

            return cb(null, got);
          }

          // Create route_points without the point at (index + 1) in the original array
          const reducedPoints = [...routePoints];
          reducedPoints.splice(index + 1, 1); // +1 because we want to remove from interior

          const routeWaypoints = reducedPoints.map(rp => ({
            lon: rp.location[0],
            lat: rp.location[1]
          }));

          const routeParams = {
            output: 'json',
            steps: 'true',
            geometries: params.geometries,
            overview: 'full'
          };

          this.requestRoute(routeWaypoints, [], [], routeParams, (routeErr, routeRes, routeBody) => {
            if (routeErr) {
              // Route failed, which means removing this point is necessary
              testedCount++;
              return testRemoval(index + 1);
            }

            let routeJson;
            try {
              routeJson = JSON.parse(routeBody);
            } catch (parseErr) {
              testedCount++;
              return testRemoval(index + 1);
            }

            if (routeJson.code !== 'Ok') {
              // Route failed, point was necessary
              testedCount++;
              return testRemoval(index + 1);
            }

            const routeGeometry = routeJson.routes[0].geometry;

            let routeCoords;
            if (isGeoJSON) {
              routeCoords = routeGeometry.coordinates;
            } else {
              const precision = params.geometries === 'polyline6' ? 6 : 5;
              routeCoords = polyline.decode(routeGeometry, precision);
            }
            const normalizedRoute = normalizeGeometry(routeCoords, isGeoJSON);

            // Check if the route changed
            const routeUnchanged = geometriesMatch(normalizedOriginal, normalizedRoute);

            if (routeUnchanged) {
              // Route didn't change when we removed this point, so set is not minimal
              allRemovalsChangeRoute = false;
              failedRemovalIndex = index;
            }

            testedCount++;
            testRemoval(index + 1);
          });
        };

        testRemoval(0);
      });
    };

    this.processRowsAndDiff(table, testRow, callback);
  });
});

When(/^I plan a trip with route-points I should verify sufficiency and minimality$/, function (table, callback) {
  this.reprocessAndLoadData((e) => {
    if (e) return callback(e);

    const testRow = (row, ri, cb) => {
      const waypoints = [];
      row.waypoints.split(',').forEach((n) => {
        const node = this.findNodeByName(n.trim());
        if (!node) throw new Error(`*** unknown waypoint node "${n.trim()}"`);
        waypoints.push(node);
      });

      const params = Object.assign({}, this.queryParams);
      params['route-points'] = 'true';
      params.steps = 'true';
      params.geometries = params.geometries || 'geojson';
      params.overview = 'full';

      if (row.source) params.source = row.source;
      if (row.destination) params.destination = row.destination;
      if (row.hasOwnProperty('roundtrip')) params.roundtrip = row.roundtrip;

      this.requestTrip(waypoints, params, (err, tripRes, tripBody) => {
        if (err) return cb(err);

        let tripJson;
        try {
          tripJson = JSON.parse(tripBody);
        } catch (parseErr) {
          return cb(new Error(`Failed to parse trip response: ${parseErr.message}`));
        }

        if (tripJson.code !== 'Ok') {
          return cb(null, {
            waypoints: row.waypoints,
            code: tripJson.code,
            sufficient: 'N/A',
            minimal: 'N/A'
          });
        }

        if (!tripJson.route_points || !tripJson.route_points[0]) {
          return cb(new Error('No route_points in trip response'));
        }

        const tripGeometry = tripJson.trips[0].geometry;
        const routePoints = tripJson.route_points[0];
        const isGeoJSON = params.geometries === 'geojson';

        let originalTripCoords;
        if (isGeoJSON) {
          originalTripCoords = tripGeometry.coordinates;
        } else {
          const precision = params.geometries === 'polyline6' ? 6 : 5;
          originalTripCoords = polyline.decode(tripGeometry, precision);
        }
        const normalizedOriginal = normalizeGeometry(originalTripCoords, isGeoJSON);

        // Test sufficiency first
        const routeWaypoints = routePoints.map(rp => ({
          lon: rp.location[0],
          lat: rp.location[1]
        }));

        const routeParams = {
          output: 'json',
          steps: 'true',
          geometries: params.geometries,
          overview: 'full'
        };

        this.requestRoute(routeWaypoints, [], [], routeParams, (routeErr, routeRes, routeBody) => {
          if (routeErr) return cb(routeErr);

          let routeJson;
          try {
            routeJson = JSON.parse(routeBody);
          } catch (parseErr) {
            return cb(new Error(`Failed to parse route response: ${parseErr.message}`));
          }

          let sufficient = false;
          if (routeJson.code === 'Ok') {
            const routeGeometry = routeJson.routes[0].geometry;
            let routeCoords;
            if (isGeoJSON) {
              routeCoords = routeGeometry.coordinates;
            } else {
              const precision = params.geometries === 'polyline6' ? 6 : 5;
              routeCoords = polyline.decode(routeGeometry, precision);
            }
            const normalizedRoute = normalizeGeometry(routeCoords, isGeoJSON);
            sufficient = geometriesMatch(normalizedOriginal, normalizedRoute);
          }

          // Now test minimality
          const interiorPoints = routePoints.slice(1, -1);
          let minimal = true;

          const testMinimality = (index) => {
            if (index >= interiorPoints.length) {
              const got = {
                waypoints: row.waypoints,
                code: 'Ok',
                sufficient: sufficient ? 'yes' : 'no',
                minimal: minimal ? 'yes' : 'no',
                route_points_count: routePoints.length
              };

              for (const key in row) {
                if (this.FuzzyMatch.match(String(got[key]), row[key])) {
                  got[key] = row[key];
                }
              }

              return cb(null, got);
            }

            const reducedPoints = [...routePoints];
            reducedPoints.splice(index + 1, 1);

            const reducedWaypoints = reducedPoints.map(rp => ({
              lon: rp.location[0],
              lat: rp.location[1]
            }));

            this.requestRoute(reducedWaypoints, [], [], routeParams, (rErr, rRes, rBody) => {
              if (rErr) {
                return testMinimality(index + 1);
              }

              let rJson;
              try {
                rJson = JSON.parse(rBody);
              } catch (pErr) {
                return testMinimality(index + 1);
              }

              if (rJson.code === 'Ok') {
                const rGeom = rJson.routes[0].geometry;
                let rCoords;
                if (isGeoJSON) {
                  rCoords = rGeom.coordinates;
                } else {
                  const precision = params.geometries === 'polyline6' ? 6 : 5;
                  rCoords = polyline.decode(rGeom, precision);
                }
                const normalizedR = normalizeGeometry(rCoords, isGeoJSON);

                if (geometriesMatch(normalizedOriginal, normalizedR)) {
                  minimal = false;
                }
              }

              testMinimality(index + 1);
            });
          };

          if (interiorPoints.length === 0) {
            // Trivially minimal
            const got = {
              waypoints: row.waypoints,
              code: 'Ok',
              sufficient: sufficient ? 'yes' : 'no',
              minimal: 'yes',
              route_points_count: routePoints.length
            };

            for (const key in row) {
              if (this.FuzzyMatch.match(String(got[key]), row[key])) {
                got[key] = row[key];
              }
            }

            return cb(null, got);
          }

          testMinimality(0);
        });
      });
    };

    this.processRowsAndDiff(table, testRow, callback);
  });
});
