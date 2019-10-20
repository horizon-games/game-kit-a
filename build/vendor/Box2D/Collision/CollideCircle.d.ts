import { Transform } from '../Common/Math';
import { Manifold } from './Collision';
import { CircleShape } from './Shapes/CircleShape';
import { PolygonShape } from './Shapes/PolygonShape';
export declare function CollideCircles(manifold: Manifold, circleA: CircleShape, xfA: Transform, circleB: CircleShape, xfB: Transform): void;
export declare function CollidePolygonAndCircle(manifold: Manifold, polygonA: PolygonShape, xfA: Transform, circleB: CircleShape, xfB: Transform): void;
