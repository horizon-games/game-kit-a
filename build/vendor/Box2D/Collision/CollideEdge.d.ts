import { Transform } from '../Common/Math';
import { Manifold } from './Collision';
import { CircleShape } from './Shapes/CircleShape';
import { EdgeShape } from './Shapes/EdgeShape';
import { PolygonShape } from './Shapes/PolygonShape';
export declare function CollideEdgeAndCircle(manifold: Manifold, edgeA: EdgeShape, xfA: Transform, circleB: CircleShape, xfB: Transform): void;
export declare function CollideEdgeAndPolygon(manifold: Manifold, edgeA: EdgeShape, xfA: Transform, polygonB: PolygonShape, xfB: Transform): void;
